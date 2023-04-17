#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from find_2d_object import objectsStamped
import tf2_ros
import tf2_geometry_msgs

def callback(data):
    for obj in data.objects.data:
        obj_pose = PoseStamped()
        obj_pose.header = data.header
        obj_pose.pose = obj.pose
        # Transform pose to base frame
        try:
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            base_frame = "base_link"
            obj_pose_base = tf_buffer.transform(obj_pose, base_frame)
            # Print object position in base frame
            rospy.loginfo("Object {} position: x={}, y={}, z={}".format(
                obj.id, obj_pose_base.pose.position.x,
                obj_pose_base.pose.position.y, obj_pose_base.pose.position.z))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Could not transform object pose to base frame: {}".format(ex))

def listener():
    rospy.init_node('object_mapper', anonymous=True)
    rospy.Subscriber("/objectsStamped", objectsStamped, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
