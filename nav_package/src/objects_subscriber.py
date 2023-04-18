#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
start = False

def callback(data):
    # Ensure there is data before trying to acess index.
    start_publisher = rospy.Publisher('/start', String, queue_size=10)
    marker_publisher = rospy.Publisher('/hazards', Marker, queue_size=10)
    global start
    if len(data.data) > 0:
        id = data.data[0]
        # Recognise Start
        if id == 'start':
            start_publisher.publish("GO")  
        # Recognise hazard and create Marker
        else:          
            marker = Marker()
            marker.id = int(id)
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0

            # Calculate the position of the image
            mapped_x, mapped_y = calculatePosition()

            # Set the position of the marker using x, y, z inputs
            marker.pose.position.x = mapped_x
            marker.pose.position.y = mapped_y
            marker.pose.position.z = 0.0
            marker_publisher.publish(marker)

            rospy.sleep(0.1)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/objects", Float32MultiArray, callback)
    rospy.Subscriber('/odom', Odometry, getCurrentPosition)
    rospy.spin()

def getCurrentPosition(data):
    global current_position 
    current_position= data.pose.pose

def calculatePosition():
    global current_position
    current_position
    return current_position.position.x, current_position.position.y


if __name__ == '__main__':
    listener()
