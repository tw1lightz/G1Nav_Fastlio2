#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from fastlio.srv import SlamReLoc  # 替换为实际服务路径
from fastlio.srv import SlamRelocCheck

class SlamRelocFromRViz:
    def __init__(self):
        self.pose_received = False
        self.pcd_path = rospy.get_param("~pcd_path", "/home/nvidia/Go2Nav2D/src/FAST_LIO_LOCALIZATION/PCD/three_floors.pcd")

        # In RViz, /initialpose.z is typically 0.0. However this repo's ICP relocalizer keeps Z
        # constant during candidate search, so a wrong Z can make relocalization appear to "do nothing".
        # Prefer sourcing Z from the current TF (global_frame -> base_frame) or a configured override.
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.use_tf_z = rospy.get_param("~use_tf_z", True)
        self.z_override = rospy.get_param("~z_override", None)  # set to a float to force Z
        self.tf_listener = tf.TransformListener()

        rospy.wait_for_service('/slam_reloc')
        self.reloc_service = rospy.ServiceProxy('/slam_reloc', SlamReLoc)

        # Optional: check whether ICP relocalization actually succeeded.
        # Note: /slam_reloc service returns immediately; success is reported via /slam_reloc_check.
        self.enable_reloc_check = rospy.get_param("~enable_reloc_check", True)
        # localizer loop_rate defaults to 1Hz in this repo; ICP can take a few seconds.
        self.reloc_check_timeout_s = float(rospy.get_param("~reloc_check_timeout_s", 10.0))
        self.reloc_check_poll_s = float(rospy.get_param("~reloc_check_poll_s", 0.25))
        self.reloc_check_service = None
        if self.enable_reloc_check:
            try:
                rospy.wait_for_service('/slam_reloc_check', timeout=1.0)
                self.reloc_check_service = rospy.ServiceProxy('/slam_reloc_check', SlamRelocCheck)
            except Exception as e:
                rospy.logwarn("/slam_reloc_check not available: %s. Will not report relocalization success.", e)

        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.loginfo("Waiting for /initialpose from RViz...")

    def pose_callback(self, msg):
        # if  self.pose_received:
        #     return  # 只处理一次
        self.pose_received = True

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        # Choose Z for relocalization
        z = position.z
        if self.z_override is not None:
            try:
                z = float(self.z_override)
                rospy.loginfo("Using z_override=%.3f (ignoring /initialpose.z)", z)
            except Exception as e:
                rospy.logwarn("Invalid z_override=%r: %s. Falling back.", self.z_override, e)
        elif self.use_tf_z:
            try:
                self.tf_listener.waitForTransform(self.global_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))
                (trans, _rot) = self.tf_listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))
                z = float(trans[2])
                rospy.loginfo("Using TF Z from %s->%s: z=%.3f", self.global_frame, self.base_frame, z)
            except Exception as e:
                rospy.logwarn("TF lookup for Z failed (%s->%s): %s. Using /initialpose.z=%.3f", self.global_frame, self.base_frame, e, z)

        rospy.loginfo("Received pose from RViz: frame=%s x=%.2f y=%.2f z=%.2f yaw=%.2f", msg.header.frame_id, position.x, position.y, z, yaw)

        try:
            resp = self.reloc_service(
                self.pcd_path,
                position.x, position.y, z,
                roll, pitch, yaw
            )
            rospy.loginfo("SlamReLoc service called successfully.")
            rospy.loginfo("Response: %s", resp)

            # Poll /slam_reloc_check for success/failure.
            if self.reloc_check_service is not None:
                deadline = rospy.Time.now() + rospy.Duration(self.reloc_check_timeout_s)
                last_status = None
                while rospy.Time.now() < deadline and not rospy.is_shutdown():
                    try:
                        check = self.reloc_check_service(False)
                        last_status = bool(check.status)
                        if last_status:
                            rospy.loginfo("Relocalization SUCCEEDED (/slam_reloc_check=true).")
                            return
                    except rospy.ServiceException as e:
                        rospy.logwarn("/slam_reloc_check call failed: %s", e)
                        break
                    rospy.sleep(self.reloc_check_poll_s)
                if last_status is False:
                    rospy.logwarn("Relocalization did NOT succeed within %.1fs (/slam_reloc_check still false).", self.reloc_check_timeout_s)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('slam_reloc_from_rviz')
    SlamRelocFromRViz()
    rospy.spin()
