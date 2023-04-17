#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class RobotController:
    start = False

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_controller', anonymous=True)

        # Initialize the robot movement publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize the LiDAR scan subscriber
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Set the initial linear and angular velocities
        self.linear_vel = 0
        self.angular_vel = 0

        # Initialize the minimum range values for front, right, and left ranges
        self.min_range_front = float('inf')
        self.min_range_right = float('inf')
        self.min_range_left = float('inf')

    def laser_callback(self, scan):
        # Calculate the minimum range values for front, right, and left ranges
        # Caluclate the number = numeber * 360 / 720
        ranges_front = scan.ranges[0:60] + scan.ranges[660:719]
        ranges_right = scan.ranges[570:600]
        ranges_left = scan.ranges[210:240]

        # Supposed to be better ones, but they are not working at the last stage now
        # ranges_right = scan.ranges[500:580]
        # ranges_left = scan.ranges[140:220]

        self.min_range_front = min(ranges_front)
        self.min_range_right = min(ranges_right)
        self.min_range_left = min(ranges_left)


     
        # If an obstacle is detected in front, stop the robot and turn left
        avoidance_distance = 0.98
        if self.min_range_front < avoidance_distance:
            self.linear_vel = 0

            if self.min_range_right > self.min_range_left:
                self.angular_vel = -0.4
            
            elif self.min_range_left > self.min_range_right:
                self.angular_vel = 0.4
            
        else:
            self.angular_vel = 0
            self.linear_vel = 0.2
            # print("front range = " + str(self.min_range_front))
            # print(self.linear_vel)
            

        # Testing the values at different places of the scan data
            # print("value at 0 = " + str(scan.ranges[0]))
            # print("value at 30 = " + str(scan.ranges[30]))
            # print("value at 100 = " + str(scan.ranges[100]))
            # print("value at 200 = " + str(scan.ranges[200]))
            # print("value at 300 = " + str(scan.ranges[300]))
            # print("value at 360 = " + str(scan.ranges[360]))
            # print("value at 550 = " + str(scan.ranges[550]))
            # print("value at 719 = " + str(scan.ranges[719]))
            # print("front range = " + str(self.min_range_front))
            # print("left range = " + str(self.min_range_left))
            # print("right range = " + str(self.min_range_right))   

        # # For testing
        # else:
        #     self.linear_vel = 0.2
        #     self.angular_vel = 0
        #     print("value at 0 = " + str(scan.ranges[0]))
        #     print("value at 360 = " + str(scan.ranges[360]))
        #     print("left range = " + str(self.min_range_left))
        #     print("right range = " + str(self.min_range_right))
        #     print("value at 719 = " + str(scan.ranges[719]))



    def run(self):
        # Set the control rate to 10 Hz
        rate = rospy.Rate(10)

        # Loop until the ROS node is shutdown
        while not rospy.is_shutdown():
            # Create a new Twist message and set the linear and angular velocities
            twist = Twist()
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel

            # Publish the Twist message to move the robot
            self.pub.publish(twist)

            # Wait for the control rate
            rate.sleep()


def callback(data):
    print(data.data)
    if data.data == "GO":
        print("RUNNING CONTROLLER")
        controller.run()

def listener():
    rospy.Subscriber("/start", String, callback)
    rospy.spin()

if __name__ == '__main__':
    # Create a new RobotController object and run the control loop
    controller = RobotController()
    listener()
