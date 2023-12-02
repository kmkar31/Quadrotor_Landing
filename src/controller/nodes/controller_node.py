#!/usr/bin/python3

import rospy
from msgs.msg import StateFbk, CtrlCmd
from std_msgs.msg import Header
import numpy as np
from scripts.CF_controller import MPCWrapper, PIDWrapper
from scripts.StateMachine import StateMachine

def process_CFfbk(msg, args):
    #print(msg.state, "\n")
    publisher = args[0]
    Controller = args[1]
    tstart = args[2]
    tnow = rospy.Time.now()
    y0 = np.reshape(np.array(msg.state),(-1,1))
    u = Controller.NextMove((tnow-tstart).to_sec(), y0)
    pub_msg = CtrlCmd()
    pub_msg.header = Header(stamp=tnow, frame_id="CF_state")
    pub_msg.u = u
    pub_msg.name = "CrazyFlie Control Command"
    publisher.publish(pub_msg)

def process_TBfbk(msg, args):
    FSM = args[0]
    tstart = args[1]
    if len(msg.state) != 0: 
        y0 = np.reshape(np.array(msg.state),(-1))
        t = (rospy.Time.now() - tstart).to_sec()
        #print("Controller",y0)
        FSM.setReference(y0,t)

def InitializeStateMachine():
    
    CtrlFreq = rospy.get_param('/ControlFrequency',50)
    T = rospy.get_param('/EndTime',60)
    TBref_time = [i/CtrlFreq for i in range(T*CtrlFreq)]

    FSM = StateMachine()
    
    #Other Tumbller Paths
    #MPC.setTumbllerPath([0.6*np.cos(np.pi*t/10) for t in TBref_time], [0.6*np.sin(np.pi*t/10) for t in TBref_time], TBref_time) # reference is a dictionary
    #MPC.setTumbllerPath([0.075*t for t in TBref_time], [0.09*t for t in TBref_time], TBref_time) # reference is a dictionary
    #MPC.setTumbllerPath([-1+0.125*(t) for t in TBref_time], [0.01 for t in TBref_time], TBref_time)
    #MPC.setTumbllerPath([0.6*np.cos(0.25*t) for t in TBref_time], [0.3*np.sin(0.5*t) for t in TBref_time], TBref_time)

    return FSM
    

# Initialize Node
rospy.init_node("controller_node")
startTime = rospy.Time.now()

# Initialize Controller 
FiniteStateMachine = InitializeStateMachine()

# CrazyFlie Communication

CF_CtrlPublisher = rospy.Publisher('/CF_Ctrl',CtrlCmd, queue_size=64)
print("Sending CrazyFlie Controller Commands")

CF_FbkListener = rospy.Subscriber('/CF_State_Feedback',StateFbk, process_CFfbk, (CF_CtrlPublisher, FiniteStateMachine, startTime))
print("Listening to CrazyFlie State Feedback")

# Tumbller Communication

TB_CtrlPublisher = rospy.Publisher('/TB_Ctrl',CtrlCmd, queue_size=64)
print("Sending Tumbller Controller Commands")

TB_FbkListener = rospy.Subscriber('/TB_State_Feedback',StateFbk, process_TBfbk, (FiniteStateMachine, startTime))
print("Listening to Tumbller State Feedback")

rate = rospy.Rate(100)
rospy.spin()