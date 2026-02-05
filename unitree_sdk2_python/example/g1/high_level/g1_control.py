#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import sys
import time
import math

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class CmdVelController:
    def __init__(self, network_interface):
        # 初始化 Unitree SDK
        rospy.loginfo("Initializing Unitree LocoClient...")
        ChannelFactoryInitialize(0, network_interface)

        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        self.has_path = False  # 标志位：是否有全局路径

        # 可调参数（尽量用私有参数，避免污染全局命名空间）
        self.cmd_buffer_size = int(rospy.get_param("~cmd_buffer_size", 5))
        self.sdk_call_interval = float(rospy.get_param("~sdk_call_interval", 0.5))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 1.0))
        self.command_duration = float(rospy.get_param("~command_duration", 1.0))

        # 机器人最小可运动速度（经验值：你测出来线速度>=0.2 或角速度>=0.3 才会动）
        self.enable_min_speed = bool(rospy.get_param("~enable_min_speed", True))
        self.min_lin_speed = float(rospy.get_param("~min_lin_speed", 0.2))
        self.min_ang_speed = float(rospy.get_param("~min_ang_speed", 0.3))
        self.lin_deadband = float(rospy.get_param("~lin_deadband", 0.02))
        self.ang_deadband = float(rospy.get_param("~ang_deadband", 0.02))
        self.max_lin_speed = float(rospy.get_param("~max_lin_speed", 1.0))
        self.max_ang_speed = float(rospy.get_param("~max_ang_speed", 0.6))

        # 倒退策略：导航中常需要少量倒退来“回正/脱困”
        # 注意：G1存在最小可运动速度阈值，因此倒退也需要单独的最小速度设置
        self.allow_backwards = bool(rospy.get_param("~allow_backwards", True))
        self.back_deadband = float(rospy.get_param("~back_deadband", 0.02))
        self.min_backwards_speed = float(rospy.get_param("~min_backwards_speed", 0.2))
        self.max_backwards_speed = float(rospy.get_param("~max_backwards_speed", 0.3))

        # 角速度符号保持：避免规划器在小角速度附近左右反复切换时，被映射成 ±min_ang_speed 导致抖动
        self.ang_sign_hold_sec = float(rospy.get_param("~ang_sign_hold_sec", 0.8))
        self._last_wz_sign = 0
        self._last_wz_sign_ts = 0.0

        # 命令缓冲（用于平滑）
        self.vx_buffer = []
        self.vy_buffer = []
        self.wz_buffer = []

        # 当前缓冲的命令（用于平均）
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        self.last_cmd_time = rospy.Time(0)

        # 订阅 /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("Subscribed to /cmd_vel")

        # 订阅全局路径
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)
        rospy.loginfo("Subscribed to global path topic /move_base/GlobalPlanner/plan")
        
        # 创建定时器，定期发送平均后的命令到SDK
        self.timer = rospy.Timer(rospy.Duration(self.sdk_call_interval), self.send_to_sdk)

    def path_callback(self, msg: Path):
        if len(msg.poses) > 0:
            if not self.has_path:
                rospy.loginfo("Global path received, robot can start moving.")
            self.has_path = True
        else:
            if self.has_path:
                rospy.logwarn("Global path is empty.")
            self.has_path = False

    def cmd_vel_callback(self, msg: Twist):
        """接收所有cmd_vel消息并缓冲，不丢弃"""
        vx = msg.linear.x      # 前后移动
        vy = msg.linear.y      # 横向移动
        wz = msg.angular.z     # 旋转

        self.last_cmd_time = rospy.Time.now()

        # 检查是否有有效路径（可选：如果你想完全不依赖路径，可以注释掉这段）
        if not self.has_path:
            # 仅在有非零命令时警告
            if vx != 0.0 or vy != 0.0 or wz != 0.0:
                rospy.logwarn_throttle(2.0, "Global path not received. Ignoring cmd_vel.")
            return

        # 将命令加入缓冲区
        self.vx_buffer.append(vx)
        self.vy_buffer.append(vy)
        self.wz_buffer.append(wz)
        
        # 保持缓冲区大小
        if len(self.vx_buffer) > self.cmd_buffer_size:
            self.vx_buffer.pop(0)
            self.vy_buffer.pop(0)
            self.wz_buffer.pop(0)
        
        # 计算平均值作为当前命令
        if len(self.vx_buffer) > 0:
            self.current_vx = sum(self.vx_buffer) / len(self.vx_buffer)
            self.current_vy = sum(self.vy_buffer) / len(self.vy_buffer)
            self.current_wz = sum(self.wz_buffer) / len(self.wz_buffer)

    def _apply_deadband_and_min_speed(self, vx: float, vy: float, wz: float):
        """将规划器输出映射为机器人可执行的速度。

        - 先做死区（过滤小抖动）
        - 再做“最小可运动速度”映射（保持方向不变）
        - 最后做限幅
        """
        # 线速度：用矢量幅值做死区/最小速度，保持vx/vy方向
        lin_mag = math.hypot(vx, vy)
        if lin_mag < self.lin_deadband:
            vx, vy = 0.0, 0.0
        else:
            # 倒退：单独的死区/最小速度/限幅，避免“小负速度”被映射成固定的大后退或被清零
            if vx < 0.0:
                if not self.allow_backwards:
                    vx, vy = 0.0, 0.0
                else:
                    if abs(vx) < self.back_deadband:
                        vx, vy = 0.0, 0.0
                    else:
                        # 仅对 x 轴倒退做最小速度映射（diff-drive 的 vy 通常为 0）
                        if self.enable_min_speed and abs(vx) < self.min_backwards_speed:
                            vx = -self.min_backwards_speed
                        vx = max(vx, -abs(self.max_backwards_speed))
                        # 倒退时不再按 lin_mag 做整体缩放，避免把 vy 也放大
            else:
                # 前进：按矢量幅值做最小速度映射，保持方向
                if self.enable_min_speed and lin_mag < self.min_lin_speed:
                    scale = self.min_lin_speed / max(lin_mag, 1e-6)
                    vx *= scale
                    vy *= scale

        # 角速度：单独做死区/最小速度
        if abs(wz) < self.ang_deadband:
            wz = 0.0
        elif self.enable_min_speed and abs(wz) < self.min_ang_speed:
            requested_sign = 1 if wz > 0 else -1
            now = time.monotonic()
            # 如果短时间内符号反复切换，则保持上一次符号
            if self._last_wz_sign != 0 and requested_sign != self._last_wz_sign and (now - self._last_wz_sign_ts) < self.ang_sign_hold_sec:
                requested_sign = self._last_wz_sign
            else:
                self._last_wz_sign = requested_sign
                self._last_wz_sign_ts = now

            wz = self.min_ang_speed if requested_sign > 0 else -self.min_ang_speed

        # 限幅
        lin_mag = math.hypot(vx, vy)
        if lin_mag > self.max_lin_speed and lin_mag > 1e-6:
            scale = self.max_lin_speed / lin_mag
            vx *= scale
            vy *= scale
        wz = max(-self.max_ang_speed, min(self.max_ang_speed, wz))

        return vx, vy, wz

    def send_to_sdk(self, event):
        """定时器回调：定期发送平均后的命令到SDK"""
        raw_vx = self.current_vx
        raw_vy = self.current_vy
        raw_wz = self.current_wz
        
        # 如果缓冲区为空（没收到任何命令），跳过
        if len(self.vx_buffer) == 0:
            return

        # 如果长时间没收到cmd_vel，强制停住（防止上一次速度被“卡住”）
        if self.last_cmd_time != rospy.Time(0) and (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            self.vx_buffer.clear()
            self.vy_buffer.clear()
            self.wz_buffer.clear()
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = 0.0
            raw_vx, raw_vy, raw_wz = 0.0, 0.0, 0.0

        vx, vy, wz = self._apply_deadband_and_min_speed(raw_vx, raw_vy, raw_wz)
        rospy.loginfo(
            f"Sending cmd (raw_avg): vx={raw_vx:.3f}, vy={raw_vy:.3f}, wz={raw_wz:.3f} -> "
            f"(mapped): vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}"
        )
        try:
            # 直接调用SetVelocity以获得返回码（Move/StopMove在SDK里不返回code）
            if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
                code = self.sport_client.SetVelocity(0.0, 0.0, 0.0, self.command_duration)
                rospy.loginfo(f"SetVelocity stop code: {code}")
            else:
                code = self.sport_client.SetVelocity(vx, vy, wz, self.command_duration)
                rospy.loginfo(f"SetVelocity code: {code}")
        except Exception as e:
            rospy.logerr(f"Failed to send Move command: {e}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: rosrun your_package cmd_vel_control.py networkInterface")
        sys.exit(-1)

    network_interface = sys.argv[1]

    rospy.init_node("unitree_cmd_vel_controller", anonymous=False)
    rospy.logwarn("Make sure the robot is in a safe environment before sending cmd_vel commands!")

    controller = CmdVelController(network_interface)

    rospy.spin()
