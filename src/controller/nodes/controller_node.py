#!/usr/bin/python3

import rospy
from msgs.msg import StateFbk, CtrlCmd
from std_msgs.msg import Header
import numpy as np
#from src.cf_controller import command




def send_CFctrl(publisher):
    msg = CtrlCmd()
    msg.header = Header(stamp=rospy.Time.now(), frame_id="CF_state")
    u = [0,0,0.3,0.0]
    msg.u = u
    msg.name = "CrazyFlie Control Command"
    publisher.publish(msg)

def send_TBctrl(publisher):
    msg = CtrlCmd()
    msg.header = Header(stamp=rospy.Time.now(), frame_id="CF_state")
    u = np.zeros(2)
    msg.u = u
    msg.name = "Tumbller Control Command"
    publisher.publish(msg)

def process_CFfbk(msg):
    print("CF State Feedback",msg.state, "\n")
    #pass


def process_TBfbk(msg):
    #print("Tumbller State Feedabck",msg.state)
    pass



# Initialize Node
rospy.init_node("controller_node")

# CrazyFlie Communication

CF_CtrlPublisher = rospy.Publisher('/CF_Ctrl',CtrlCmd, queue_size=64)
print("Sending CrazyFlie Controller Commands")

CF_FbkListener = rospy.Subscriber('/CF_State_Feedback',StateFbk, process_CFfbk)
print("Listening to CrazyFlie State Feedback")

# Tumbller Communication

TB_CtrlPublisher = rospy.Publisher('/TB_Ctrl',CtrlCmd, queue_size=64)
print("Sending Tumbller Controller Commands")

TB_FbkListener = rospy.Subscriber('/TB_State_Feedback',StateFbk, process_TBfbk)
print("Listening to Tumbller State Feedback")

rate = rospy.Rate(100)

while not(rospy.is_shutdown()):
    send_CFctrl(CF_CtrlPublisher)
    send_TBctrl(TB_CtrlPublisher)
    rate.sleep()

rospy.spin()