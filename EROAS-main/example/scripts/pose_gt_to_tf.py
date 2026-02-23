#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PoseGTToTF:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/rexrov/pose_gt")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "rexrov/base_link")
        self.force_direct = bool(rospy.get_param("~force_direct", True))
        self.wait_for_pose_gt = bool(rospy.get_param("~wait_for_pose_gt", True))
        self.wait_timeout = float(rospy.get_param("~wait_timeout", 10.0))
        self.footprint_frame_id = self._normalize_optional_frame(
            rospy.get_param("~footprint_frame_id", "rexrov/base_footprint")
        )
        self.stabilized_frame_id = self._normalize_optional_frame(
            rospy.get_param("~stabilized_frame_id", "rexrov/base_stabilized")
        )
        self.publish_roll_pitch = bool(rospy.get_param("~publish_roll_pitch", True))

        self.br = tf.TransformBroadcaster()
        if self.wait_for_pose_gt:
            try:
                rospy.wait_for_message(self.odom_topic, Odometry, timeout=self.wait_timeout)
                rospy.loginfo("pose_gt_to_tf: pose_gt available, start TF publishing")
            except Exception:
                rospy.logwarn("pose_gt_to_tf: wait_for_pose_gt timeout, will still subscribe")
        rospy.Subscriber(self.odom_topic, Odometry, self._cb, queue_size=1)

    @staticmethod
    def _normalize_optional_frame(val):
        if val is None:
            return ""
        if isinstance(val, str) and val.strip().lower() in ("", "none", "null", "false", "0"):
            return ""
        return val

    def _cb(self, msg):
        stamp = msg.header.stamp
        if stamp.to_sec() <= 0.0:
            stamp = rospy.Time.now()

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        # Publish transforms using tf (ros1)
        if self.force_direct or (not self.footprint_frame_id and not self.stabilized_frame_id):
            tf_base = TransformStamped()
            tf_base.header.stamp = stamp
            tf_base.header.frame_id = self.frame_id
            tf_base.child_frame_id = self.child_frame_id
            tf_base.transform.translation.x = pos.x
            tf_base.transform.translation.y = pos.y
            tf_base.transform.translation.z = pos.z
            if self.publish_roll_pitch:
                q = quaternion_from_euler(roll, pitch, yaw)
            else:
                q = quaternion_from_euler(0.0, 0.0, yaw)
            tf_base.transform.rotation.x = q[0]
            tf_base.transform.rotation.y = q[1]
            tf_base.transform.rotation.z = q[2]
            tf_base.transform.rotation.w = q[3]
            self.br.sendTransform(
                (tf_base.transform.translation.x,
                 tf_base.transform.translation.y,
                 tf_base.transform.translation.z),
                (tf_base.transform.rotation.x,
                 tf_base.transform.rotation.y,
                 tf_base.transform.rotation.z,
                 tf_base.transform.rotation.w),
                stamp,
                tf_base.child_frame_id,
                tf_base.header.frame_id,
            )
            rospy.loginfo_throttle(
                5.0,
                "pose_gt_to_tf publishing %s -> %s",
                tf_base.header.frame_id,
                tf_base.child_frame_id,
            )
            return

        parent_frame = self.frame_id
        if self.footprint_frame_id:
            tf_foot = TransformStamped()
            tf_foot.header.stamp = stamp
            tf_foot.header.frame_id = self.frame_id
            tf_foot.child_frame_id = self.footprint_frame_id
            tf_foot.transform.translation.x = pos.x
            tf_foot.transform.translation.y = pos.y
            tf_foot.transform.translation.z = 0.0
            q = quaternion_from_euler(0.0, 0.0, yaw)
            tf_foot.transform.rotation.x = q[0]
            tf_foot.transform.rotation.y = q[1]
            tf_foot.transform.rotation.z = q[2]
            tf_foot.transform.rotation.w = q[3]
            self.br.sendTransform(
                (tf_foot.transform.translation.x,
                 tf_foot.transform.translation.y,
                 tf_foot.transform.translation.z),
                (tf_foot.transform.rotation.x,
                 tf_foot.transform.rotation.y,
                 tf_foot.transform.rotation.z,
                 tf_foot.transform.rotation.w),
                stamp,
                tf_foot.child_frame_id,
                tf_foot.header.frame_id,
            )
            parent_frame = self.footprint_frame_id

        if self.stabilized_frame_id:
            tf_stab = TransformStamped()
            tf_stab.header.stamp = stamp
            tf_stab.header.frame_id = parent_frame
            tf_stab.child_frame_id = self.stabilized_frame_id
            tf_stab.transform.translation.x = 0.0
            tf_stab.transform.translation.y = 0.0
            tf_stab.transform.translation.z = pos.z
            tf_stab.transform.rotation.x = 0.0
            tf_stab.transform.rotation.y = 0.0
            tf_stab.transform.rotation.z = 0.0
            tf_stab.transform.rotation.w = 1.0
            self.br.sendTransform(
                (tf_stab.transform.translation.x,
                 tf_stab.transform.translation.y,
                 tf_stab.transform.translation.z),
                (tf_stab.transform.rotation.x,
                 tf_stab.transform.rotation.y,
                 tf_stab.transform.rotation.z,
                 tf_stab.transform.rotation.w),
                stamp,
                tf_stab.child_frame_id,
                tf_stab.header.frame_id,
            )
            parent_frame = self.stabilized_frame_id

        tf_base = TransformStamped()
        tf_base.header.stamp = stamp
        tf_base.header.frame_id = parent_frame
        tf_base.child_frame_id = self.child_frame_id
        tf_base.transform.translation.x = 0.0
        tf_base.transform.translation.y = 0.0
        tf_base.transform.translation.z = 0.0
        if self.publish_roll_pitch:
            q = quaternion_from_euler(roll, pitch, 0.0)
        else:
            q = quaternion_from_euler(0.0, 0.0, 0.0)

        tf_base.transform.rotation.x = q[0]
        tf_base.transform.rotation.y = q[1]
        tf_base.transform.rotation.z = q[2]
        tf_base.transform.rotation.w = q[3]
        self.br.sendTransform(
            (tf_base.transform.translation.x,
             tf_base.transform.translation.y,
             tf_base.transform.translation.z),
            (tf_base.transform.rotation.x,
             tf_base.transform.rotation.y,
             tf_base.transform.rotation.z,
             tf_base.transform.rotation.w),
            stamp,
            tf_base.child_frame_id,
            tf_base.header.frame_id,
        )

        rospy.loginfo_throttle(
            5.0,
            "pose_gt_to_tf publishing %s -> %s",
            tf_base.header.frame_id,
            tf_base.child_frame_id,
        )


def main():
    rospy.init_node("pose_gt_to_tf")
    PoseGTToTF()
    rospy.spin()


if __name__ == "__main__":
    main()
