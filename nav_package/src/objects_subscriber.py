#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


start = False

def callback(data):
    # ensure there is data before trying to acess index.
    print()
    publisher = rospy.Publisher('/start', String, queue_size=10)
    global start
    if len(data.data) > 0:
        id = data.data[0]
        # Recognise Start
        print(id)
        if id == 13:
            publisher.publish("GO")            
    else:
        print("Couldnt see anything")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/objects", Float32MultiArray, callback)
    # pub = rospy.Publisher("hazards")
    rospy.spin()

if __name__ == '__main__':
    listener()