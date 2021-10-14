#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from enum import Enum
from controller import PID

class ControllerState(Enum):
    IDLE = 1
    RUNNING = 2
    FINISHED = 3


class ParrotBebop:
    state = ControllerState.IDLE
    def __init__(self, pose, ref, dt):


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


        self.ref = ref
        self.pose = pose
        self.combine_controller = (self.x_controller.calculate, self.y_controller.calculate, self.z_controller.calculate, self.yaw_controller.calculate)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.controlLoop = rospy.Timer(rospy.Duration(dt), self.calculate)

    def set_ref(self, target):
        assert isinstance(target, list), 'target pose should be a list with 4 param'
        self.ref = target

    def update_state(self, x):
        assert isinstance(x, list), 'uav state should have 4 param (x, y, z, yaw)'
        self.pose = x

    def calculate(self, timer):
        u = [0] * 4
        if self.state == ControllerState.RUNNING:
            u = [f(self.ref[i], self.pose[i])for i, f in enumerate(self.combine_controller)]
        print("[+ {}] publishing cmd_vel ".format(timer.current_real), u)
        self.publish_cmd_vel(u)

    def publish_cmd_vel(self, u):
        msg = Twist()
        msg.linear.x = u[0]
        msg.linear.y = u[1]
        msg.linear.z = u[2]
        msg.angular.z = u[3]
        self.cmd_pub.publish(msg)





if __name__ == '__main__':
    rospy.init_node('bebop_controller', anonymous=True)

    pose = [0, 0, 0, 0]
    ref = [-1, 1, 0, 0]
    drone = ParrotBebop(pose, ref, dt = 0.03) # 30 Hz
    drone.state = ControllerState.RUNNING
    rospy.spin()