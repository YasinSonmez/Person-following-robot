#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState  

CAM_WIDTH = 640
CAM_HEIGHT = 480
PI = 3.14
RX_LIM = PI / 3.6
RY_LIM = PI / 3.6
REFLECT_AXIS_X = True
REFLECT_AXIS_Y = True
e_ss_lim = 0.01
current_rx = None
current_ry = None
pan_moving = False
tilt_moving = False


def calc_ref(target, current_r, max_res, r_lim, is_reflected):
    center = max_res / 2
    diff = target - center
    new_r = (diff + center) / max_res * r_lim - r_lim / 2
    if is_reflected:
        new_r = -new_r
    try:
        new_r += current_r
    except TypeError:
        current_r = 0
    current_r = new_r
    return new_r

def detect_movement(r, y, name):
    if r is None:
        return False

    if abs(r - y) > e_ss_lim:
        print(f"{name.capitalize()} is moving. Err: {abs(r - y)}")
        return True
    
    return False
    

def target_x_cb(cmd):
    if pan_moving:
        print("Pan is busy")
        return
    global current_rx
    current_rx = calc_ref(cmd.data, current_rx, CAM_WIDTH, RX_LIM, REFLECT_AXIS_X)
    pan_ref_pub.publish(current_rx)

def target_y_cb(cmd):
    if tilt_moving:
        print("Tilt is busy")
        return
    global current_ry
    current_ry = calc_ref(cmd.data, current_ry, CAM_HEIGHT, RY_LIM, REFLECT_AXIS_Y)
    tilt_ref_pub.publish(current_ry)

def pan_state_cb(data):
    global pan_moving
    pan_moving = detect_movement(current_rx, data.process_value, "pan")

def tilt_state_cb(data):
    global tilt_moving
    tilt_moving = detect_movement(current_ry, data.process_value, "tilt")






rospy.init_node("pt_cmd")
pan_ref_pub = rospy.Publisher("/pan_controller/command", Float64, queue_size=1)
tilt_ref_pub = rospy.Publisher("/tilt_controller/command", Float64, queue_size=1)

target_x_sub = rospy.Subscriber("targetX", Float64, target_x_cb)
target_y_sub = rospy.Subscriber("targetY", Float64, target_y_cb)

pan_state_sub = rospy.Subscriber("/pan_controller/state", JointControllerState, pan_state_cb) 
tilt_state_sub = rospy.Subscriber("/tilt_controller/state", JointControllerState, tilt_state_cb) 

rospy.spin()
    
