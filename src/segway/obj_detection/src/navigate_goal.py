import rospy
from geometry_msgs import msg
from dataclasses import dataclass
from std_msgs.msg import Float64
import numpy as np
from control_msgs.msg import JointControllerState  


class Navigator():
    def __init__(self, pub):
        self.distance = np.inf
        self.pub = pub
        self.dist_low_lim = 1
        self.pan_angle = 0
        self.throttle = 0.75
        self.receive_depth_cnt = 0
        self.receive_pan_cnt = 0
        self.mode = "IDLE"

    def spin(self):
        if self.mode == "IDLE":
            print("IDLE")
            if self.got_initials():
                self.mode = "NAVIGATE_GOAL"
        if self.mode == "NAVIGATE_GOAL":
            self.navigate_goal()
            self.mode = "NAVIGATING_GOAL"

        if self.mode == "NAVIGATING_GOAL":
            print("Navigating to goal...")
            pass

    def got_initials(self):
        if self.receive_depth_cnt and self.receive_pan_cnt:
            return True
        return False

    def navigate_goal(self):
        command = msg.PoseStamped()
        command.pose.position.x = self.distance
        command.pose.orientation.z = self.pan_angle
        self.pub.publish(command)

    def pan_cb(self, data):
        self.receive_pan_cnt += 1
        self.pan_angle = data.process_value

    def depth_cb(self, data):
        self.receive_depth_cnt += 1
        self.distance = data.data - self.dist_low_lim

rospy.init_node("navigator_goal")
pub = rospy.Publisher("/goal", msg.PoseStamped, queue_size=1)

navi = Navigator(pub)
rospy.Subscriber("/targetD", Float64, callback=navi.depth_cb)
rospy.Subscriber("/pan_controller/state", JointControllerState, callback=navi.pan_cb)
while True:
    navi.spin()