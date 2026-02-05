#!/usr/bin/env python3
"""
简化版 G1 控制器，用于测试 cmd_vel 话题
去除路径依赖，可直接通过 rostopic pub 测试

使用方法：
1. python3 g1_control_test.py <网口名>
2. rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}"
"""
import rospy
from geometry_msgs.msg import Twist
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class G1ControllerTest:
    def __init__(self, network_interface):
        # 初始化 Unitree SDK
        rospy.loginfo("Initializing Unitree LocoClient...")
        ChannelFactoryInitialize(0, network_interface)

        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        
        # 频率限制：防止高频调用 SDK
        self.min_cmd_interval = 0.1  # 最小命令间隔（秒），即最高 10Hz
        self.last_cmd_time = 0.0
        
        # 上一次发送的命令
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0
        
        # 命令计数器，用于调试
        self.cmd_count = 0
        self.robot_ready = True  # 假设机器人已经就绪

        # 订阅 /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("Subscribed to /cmd_vel - Robot ready for cmd_vel!")
        rospy.loginfo("Try: rostopic pub -r 2 /cmd_vel geometry_msgs/Twist \"linear: {x: 0.1}\"")

    def cmd_vel_callback(self, msg: Twist):
        if not self.robot_ready:
            rospy.logwarn_throttle(2.0, "Robot not ready, ignoring cmd_vel")
            return
            
        self.cmd_count += 1
        
        vx = msg.linear.x      # 前后移动
        vy = msg.linear.y      # 横向移动  
        wz = msg.angular.z     # 旋转

        # 频率限制检查 - 增加到0.5秒间隔，模仿官方示例的节奏
        current_time = time.time()
        if current_time - self.last_cmd_time < 0.5:  # 改为0.5秒间隔
            rospy.logdebug(f"Cmd #{self.cmd_count} skipped (rate limit)")
            return  # 跳过，避免高频调用
        
        # 避免重复发送完全相同的命令
        if vx == self.last_vx and vy == self.last_vy and wz == self.last_wz:
            # 零速度命令总是需要发送（用于停止）
            if not (vx == 0.0 and vy == 0.0 and wz == 0.0):
                rospy.logdebug(f"Cmd #{self.cmd_count} skipped (duplicate)")
                return  # 跳过重复的非零命令

        rospy.loginfo(f"[{self.cmd_count}] Sending Move: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
        try:
            # 改为一次性动作模式，类似官方示例
            if vx == 0.0 and vy == 0.0 and wz == 0.0:
                # 停止命令
                result = self.sport_client.StopMove()
                rospy.loginfo(f"StopMove result: {result}")
            else:
                # 使用默认的一次性移动，不用连续模式
                result = self.sport_client.Move(vx, vy, wz)  # continous_move=False (默认)
                rospy.loginfo(f"Move result: {result}")
                
            self.last_cmd_time = current_time
            self.last_vx = vx
            self.last_vy = vy
            self.last_wz = wz
        except Exception as e:
            rospy.logerr(f"Failed to send Move command: {e}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        print(f"Example: python3 {sys.argv[0]} enp3s0")
        sys.exit(-1)

    network_interface = sys.argv[1]

    rospy.init_node("g1_controller_test", anonymous=False)
    rospy.logwarn("===== G1 Control Test Mode =====")
    rospy.logwarn("Make sure the robot is in a safe environment!")
    rospy.logwarn("Assuming robot is already in motion mode")

    controller = G1ControllerTest(network_interface)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down G1 controller test")
        # 停止机器人
        try:
            controller.sport_client.StopMove()
        except:
            pass