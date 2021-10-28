#!/usr/bin/python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import math
count = 0
def getMarkerWindow(x,y,z,r,p,yaw):

    myMarker = Marker()
    myMarker.header.frame_id = "map"
    myMarker.header.seq = 1
    myMarker.header.stamp    = rospy.get_rostime()
    myMarker.ns = "bebop_target"
    myMarker.id = 0
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
    myMarker.color=ColorRGBA(0.41, 0.41, 0.41, 1)
    myMarker.scale.x = 1.0
    myMarker.scale.y = 1.0
    myMarker.scale.z = 1.0

    # use mesh
    myMarker.type = myMarker.MESH_RESOURCE
    myMarker.mesh_resource = 'package://bebop_controller/config/quad_model.stl'

    return myMarker

def point_trajectory(x,y,z,r,p,yaw):
    global count
    count += 1
    myMarker = Marker()
    myMarker.header.frame_id = "map"
    myMarker.header.seq = 1
    myMarker.header.stamp    = rospy.get_rostime()
    myMarker.ns = "bebop_trajectory"
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
    myMarker.color=ColorRGBA(1, 1, 0, 0.4)
    myMarker.scale.x = 0.05
    myMarker.scale.y = 0.05
    myMarker.scale.z = 0.05
    return myMarker

def vizCallback(data):
    # TODO comment out two lines for simulation
    # msg = getMarkerWindow(data.x, data.y, data.z, 0, 0, math.pi/4)
    # marker_pub.publish(msg)
    traj = point_trajectory(data.x, data.y, data.z, 0, 0, 0)
    traj_pub.publish(traj)

def robotStateCallback(data):
    orientation_list = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    msg = getMarkerWindow(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z,
                          roll, pitch, yaw)
    marker_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('bebop_visualization', anonymous=True)
    rospy.Subscriber('/bebop/trajectory', Point, vizCallback)
    rospy.Subscriber('/vicon/bebop/bebop', TransformStamped, robotStateCallback)
    marker_pub = rospy.Publisher('/bebop/target', Marker, queue_size=1)
    traj_pub = rospy.Publisher('/bebop/trajectory', Marker, queue_size=1)
    rospy.spin()
