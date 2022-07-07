#!/usr/bin/python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
#from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def robotStateCallback(data):
    #msg = Odometry()
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.get_rostime()
    #msg.child_frame_id = '/vicon/bebop/bebop'

    # position
    msg.pose.pose.position.x = data.transform.translation.x
    msg.pose.pose.position.y = data.transform.translation.y
    msg.pose.pose.position.z = data.transform.translation.z
    # orientation
    msg.pose.pose.orientation.x = data.transform.rotation.x
    msg.pose.pose.orientation.y = data.transform.rotation.y
    msg.pose.pose.orientation.z = data.transform.rotation.z
    msg.pose.pose.orientation.w = data.transform.rotation.w
    
    pose_pub.publish(msg)

    # publish baslink tf
    msg.pose.pose.orientation.x = 0
    msg.pose.pose.orientation.y = msg.pose.pose.orientation.z = 0
    msg.pose.pose.orientation.w = 1
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    "base_link",
                    "map")

def getMarkerWindow(x,y,z,r,p,yaw):

    myMarker = Marker()
    myMarker.header.frame_id = "map"
    myMarker.header.seq = 1
    myMarker.header.stamp    = rospy.get_rostime()
    myMarker.ns = "bebop_model"
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

def ekf_filter(data):
    data = data.pose.pose.position
    msg = getMarkerWindow(data.x, data.y, data.z, 0, 0, math.pi/4)
    marker_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('bebop_visualization', anonymous=True)
    rospy.Subscriber('/vicon/bebop/bebop', TransformStamped, robotStateCallback)
    rospy.Subscriber('/odometry/filtered', Odometry, ekf_filter)
    pose_pub = rospy.Publisher('/bebop/pose', PoseWithCovarianceStamped, queue_size=10)
    marker_pub = rospy.Publisher('/bebop/model', Marker, queue_size=1)
    rospy.spin()
