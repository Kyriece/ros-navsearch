#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


start = False


def callback(data):
    # ensure there is data before trying to acess index.
    start_publisher = rospy.Publisher('/start', String, queue_size=10)
    marker_publisher = rospy.Publisher('/hazards', Marker, queue_size=10)
    global start
    if len(data.data) > 0:
        id = data.data[0]
        # Recognise Start
        if id == 'start':
            start_publisher.publish("GO")  
        # Recognised hazard
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
            # mapped_x, mapped_y = calculatePosition()
            mapped_x = 1
            mapped_y = 1

            # Set the position of the marker using x, y, z inputs
            marker.pose.position.x = mapped_x
            marker.pose.position.y = mapped_y
            marker.pose.position.z = 0.0
            marker_publisher.publish(marker)

            rospy.sleep(0.1)
    else:
        print("Couldnt see anything")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/objects", Float32MultiArray, callback)
    rospy.Subscriber("/marker_array", MarkerArray)
    # pub = rospy.Publisher("hazards")
    rospy.spin()

def calculatePosition():
    # Also struggling to get current position ;_;
    # tf_buffer = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buffer)
    # trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time())
    # ros_x = trans.transform.translation.x
    # ros_y = trans.transform.translation.y
    # return ros_x, ros_y
    return 0, 0
    # Was going to calculate the manhatten distance between current marker and possible new marker (current position)
    # But need the distance to the marker for that!
    # for marker in msg.markers:
    #     if marker.id == desired_id:
    #         # Get position of the marker
    #         marker_x = marker.pose.position.x
    #         marker_y = marker.pose.position.y





if __name__ == '__main__':
    listener()