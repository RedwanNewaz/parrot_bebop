#!/usr/bin/python3
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA, Empty
from enum import Enum
from controller import PID
from geometry_msgs.msg import Twist
import tf

step_incr = 0.01
dead_zone = 0.3


class ControllerState(Enum):
    IDLE = 1
    RUNNING = 2
    FINISHED = 3

class ParrotBebopController:
    state = ControllerState.IDLE

    def __init__(self, dt):
        x_Kp = rospy.get_param('~x_axis/Kp')
        x_Kd = rospy.get_param('~x_axis/Kd')
        x_Ki = rospy.get_param('~x_axis/Ki')
        print("[+ x] axis controller gain", x_Kp, x_Kd, x_Ki)

        y_Kp = rospy.get_param('~y_axis/Kp')
        y_Kd = rospy.get_param('~y_axis/Kd')
        y_Ki = rospy.get_param('~y_axis/Ki')
        print("[+ y] axis controller gain", y_Kp, y_Kd, y_Ki)

        z_Kp = rospy.get_param('~z_axis/Kp')
        z_Kd = rospy.get_param('~z_axis/Kd')
        z_Ki = rospy.get_param('~z_axis/Ki')
        print("[+ z] axis controller gain", z_Kp, z_Kd, z_Ki)

        yaw_Kp = rospy.get_param('~yaw_axis/Kp')
        yaw_Kd = rospy.get_param('~yaw_axis/Kd')
        yaw_Ki = rospy.get_param('~yaw_axis/Ki')
        print("[+ yaw] axis controller gain", yaw_Kp, yaw_Kd, yaw_Ki)

        self.x_controller = PID(dt, 1, -1, x_Kp, x_Kd, x_Ki)
        self.y_controller = PID(dt, 1, -1, y_Kp, y_Kd, y_Ki)
        self.z_controller = PID(dt, 1, -1, z_Kp, z_Kd, z_Ki)
        self.yaw_controller = PID(dt, 1, -1, yaw_Kp, yaw_Kd, yaw_Ki)

        self.listener = tf.TransformListener()
        self.combine_controller = (
            self.x_controller.calculate, self.y_controller.calculate, self.z_controller.calculate,
            self.yaw_controller.calculate)
        self.cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.controlLoop = rospy.Timer(rospy.Duration(dt), self.calculate)
        self.stateLoop = rospy.Timer(rospy.Duration(0.01), self.bebop_state)

        self._isPoseUpdated = False
        self._isRefUpdated = False

    def set_ref(self, target):
        assert isinstance(target, list), 'target pose should be a list with 4 param'
        self.ref = target
        self._isRefUpdated = True

    def update_state(self, x):
        assert isinstance(x, list), 'uav state should have 4 param (x, y, z, yaw)'
        self.pose = x
        self._isPoseUpdated = True
        print(f"[Bebop Pose] = ", x)

    def calculate(self, timer):

        if not self._isRefUpdated  or not self._isPoseUpdated:
            return

        if self.state == ControllerState.RUNNING:
            u = [0] * 4
            u = [f(self.ref[i], self.pose[i]) for i, f in enumerate(self.combine_controller)]
            # print("[+ {}] publishing cmd_vel ".format(timer.current_real), u)
            self.publish_cmd_vel(u)



    def bebop_state(self, event):
        try:
            (trans,rot) = self.listener.lookupTransform('/vicon/world', '/vicon/bebop/bebop', rospy.Time(0))
            yaw = 0
            self.update_state([trans[0], trans[1], trans[2], yaw])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def publish_cmd_vel(self, u):
        msg = Twist()
        msg.linear.x = u[0]
        msg.linear.y = u[1]
        msg.linear.z = u[2]
        msg.angular.z = u[3]
        self.cmd_pub.publish(msg)


class TargetHandle:
    def __init__(self, x, y, z):
        self._state = np.array([x, y, z], dtype=np.float64)
        self._takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self._land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self._drone = ParrotBebopController(dt=0.03)

    def moveX(self, dx):
        if abs(dx) < dead_zone: return
        dx = np.sign(dx) * step_incr
        incr = np.array([dx, 0, 0])
        self._state += incr

    def moveY(self, dy):
        if abs(dy) < dead_zone: return
        dy = np.sign(dy) * step_incr
        incr = np.array([0, dy, 0])
        self._state += incr

    def moveZ(self, dz):
        if abs(dz) < dead_zone: return
        dz = np.sign(dz) * step_incr
        incr = np.array([0, 0, dz])
        self._state += incr

    def __repr__(self):
        return f"[TargetHandle]: x = {self._state[0]:.3f} y = {self._state[1]:.3f} z = {self._state[2]:.3f}"



class VizTarget(TargetHandle):
    def __init__(self, x, y, z):
        super().__init__(x, y, z)
        self.targetPub = rospy.Publisher('/bebop/target', Marker, queue_size=10)
        self.dx = self.dy = self.dz = 0
        rospy.Subscriber('/joy', Joy, self.joystickCallback)
        rospy.Timer(rospy.Duration(0.01), self.loop)


    def loop(self, event):
        self.moveZ(self.dz)
        self.moveX(self.dx)
        self.moveY(-self.dy)
        marker = self.showTarget()
        self.targetPub.publish(marker)
        yaw = 0
        self._drone.set_ref([self._state[0], self._state[1], self._state[2], yaw])

    def dcodeButtons(self, data):
        msg = Empty()
        if (data.buttons[0]):
            self._drone.state = ControllerState.RUNNING
            print('Control Mode active', self)
        elif data.buttons[2]:
            print('IDLE')
            if self._drone._isPoseUpdated:
                self._state[0] = self._drone.pose[0]
                self._state[1] = self._drone.pose[1]
                self._state[2] = self._drone.pose[2]
            else:
                print("NO VICON DATA")

        elif data.buttons[3]:
            print('Takeoff')
            self._takeoff_pub.publish(msg)
        elif data.buttons[1]:
            print('Land')
            self._land_pub.publish(msg)
        # else:
        #     print(self)


    def joystickCallback(self, msg):
        self.dz = msg.axes[1]
        self.dx = msg.axes[3]
        self.dy = msg.axes[4]
        # self.loop(None)
        # print(self.dx, self.dy, self.dz)
        self.dcodeButtons(msg)


    def showTarget(self):
        myMarker = Marker()
        myMarker.header.frame_id = "map"
        myMarker.header.seq = 1
        myMarker.header.stamp    = rospy.get_rostime()
        myMarker.ns = "bebop_target"
        myMarker.id = 101
        myMarker.type = myMarker.SPHERE
        myMarker.action = myMarker.ADD
        myMarker.pose.position.x = self._state[0]
        myMarker.pose.position.y = self._state[1]
        myMarker.pose.position.z = self._state[2]
        myMarker.color=ColorRGBA(0, 1, 0, 0.4)
        myMarker.scale.x = 0.5
        myMarker.scale.y = 0.5
        myMarker.scale.z = 0.5
        return myMarker







if __name__ == '__main__':
    rospy.init_node("bebop_joy")
    print("bebop joy node started")
    viz = VizTarget(0, 0, 1)
    rospy.spin()