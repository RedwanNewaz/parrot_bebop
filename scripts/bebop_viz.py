#!/usr/bin/python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
count = 0
def getMarkerWindow(x,y,z,r,p,yaw):
    global count
    count += 1
    myMarker = Marker()
    myMarker.header.frame_id = "map"
    myMarker.header.seq = 1
    myMarker.header.stamp    = rospy.get_rostime()
    myMarker.ns = "bebop_target"
    myMarker.id = count
    myMarker.type = myMarker.SPHERE
    myMarker.action = myMarker.ADD
    myMarker.pose.position.x = x
    myMarker.pose.position.y = y
    myMarker.pose.position.z = z
    q = quaternion_from_euler(r, p, yaw)
    myMarker.pose.orientation.x=q[0]
    myMarker.pose.orientation.y=q[1]
    myMarker.pose.orientation.z=q[2]
    myMarker.pose.orientation.w=q[3]
    myMarker.color=ColorRGBA(0, 1, 0, 0.4)
    myMarker.scale.x = 0.25
    myMarker.scale.y = 0.25
    myMarker.scale.z = 0.25

    return myMarker

def vizCallback(data):
    msg = getMarkerWindow(data.x, data.y, data.z, 0, 0, 0)
    marker_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('bebop_visualization', anonymous=True)
    rospy.Subscriber('/bebop/trajectory', Point, vizCallback)
    marker_pub = rospy.Publisher('/bebop/target', Marker, queue_size=1)
    rospy.spin()
