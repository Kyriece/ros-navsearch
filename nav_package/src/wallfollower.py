#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WallFollower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('wall_follower')

        # Initialize robot movement publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize LiDAR scan subscriber
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Set the initial linear and angular velocities
        self.linear_vel = 0.2
        self.angular_vel = 0

        # Set the distance thresholds for wall following
        self.wall_distance = 0.6
        self.max_wall_distance = 1.0
        self.min_wall_distance = 0.5

    def laser_callback(self, scan):
        # Extract the range values from the LiDAR scan data
        ranges = scan.ranges

        # Find the minimum range value in the right 90-degree field of view
        right_ranges = ranges[1330:1543]
        right_min_range = min(right_ranges)

        # Find the minimum range value in the front 90-degree field of view
        front_ranges = ranges[0:80] + ranges[1832:1912]
        front_min_range = min(front_ranges)

        # Find the minimum range value in the left 90-degree field of view
        left_ranges = ranges[372:585]
        left_min_range = min(left_ranges)

        # Check if there is a wall on the right
        if right_min_range < self.wall_distance:
            # Turn left
            self.linear_vel = 0.1
            self.angular_vel = 0.5

        # Check if there is a wall on the front side
        elif front_min_range < self.wall_distance:
            # Turn right
            self.linear_vel = 0.1
            self.angular_vel = -0.5

        # Check if there is a wall on the left side
        elif left_min_range < self.wall_distance:
            # Go straight
            self.linear_vel = 0.2
            self.angular_vel = 0

        
        # If there are no walls nearby, turn randomly
        else:
            self.linear_vel = 0.1
            self.angular_vel = 1

    def run(self):
        # Set the control rate to 10 Hz
        rate = rospy.Rate(10)

        # Loop until the ROS node is shutdown
        while not rospy.is_shutdown():
            # Publish the linear and angular velocities to move the robot
            twist = Twist()
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel

            self.pub.publish(twist)

            rate.sleep()

def callback(data):
    print(data.data + "- data")
    if data.data == "GO":
        print("RUNNING CONTROLLER")
        wf.run()

def listener():
    rospy.Subscriber("/start", String, callback)
    rospy.spin()

if __name__ == '__main__':
    # Create a new WallFollower object and run the control loop
    wf = WallFollower()
    listener()