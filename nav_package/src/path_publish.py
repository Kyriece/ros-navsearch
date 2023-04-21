#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Global variable to store past poses
past_poses = []

def odom_callback(odom_msg):
    global past_poses
    past_poses.append(odom_msg.pose.pose)

def publish_path():
    global past_poses
    rospy.init_node('path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'  # Update the frame_id according to your setup

        # Append past poses to the path
        for pose in past_poses:
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            path.poses.append(pose_stamped)

        # Publish the path
        path_pub.publish(path)
        rate.sleep()

rospy.Subscriber('/odom', Odometry, odom_callback)

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass