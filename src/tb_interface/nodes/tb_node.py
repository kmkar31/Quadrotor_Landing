#!/usr/bin/python3

import rospy
from msgs.msg import StateFbk, CtrlCmd
from std_msgs.msg import Header
import numpy as np


def send_fbk(publisher):
    msg = StateFbk()
    msg.header = Header(stamp=rospy.Time.now(), frame_id="TB_state")
    state = np.zeros(6)
    msg.state = state
    msg.name = "Tumbller State Feedback"
    publisher.publish(msg)

def process_ctrl(msg):
    #print("TB Control Commands", msg.u)
    pass


# Initialize Node
rospy.init_node("cf_node")

CtrlListener = rospy.Subscriber('/TB_Ctrl',CtrlCmd, process_ctrl)
print("Listening to Controller Commands")

FbkPublisher = rospy.Publisher('/TB_State_Feedback',StateFbk, queue_size=64)
print("Sending CF State Feedback")

rate = rospy.Rate(100)

while not(rospy.is_shutdown()):
    send_fbk(FbkPublisher)
    rate.sleep()

rospy.spin()
    
    