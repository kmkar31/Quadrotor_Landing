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
    Controller = args
    print(msg.state)
    Controller.updateTargetPos(msg.state[0:2])
    #print(Controller.TargetPosition)

def InitializeController():
    
    CtrlFreq = rospy.get_param('/ControlFrequency',50)
    T = rospy.get_param('/T',60)
    TBref_time = [i/CtrlFreq for i in range(T*CtrlFreq)]
    #params = rospy.get_param("/MPCParams")
    '''MPC = MPCWrapper(params["m"], params["p"], params["q"], params["N"],
                     params["A"], params["B"], params["C"], params["Q"],
                     params["R"], params["ymin"], params["ymax"], 
                     params["umin"], params["umax"])'''
    #np.set_printoptions(threshold = np.inf)
    FSM = StateMachine(([1 for t in TBref_time], [0.01 for t in TBref_time], TBref_time))
    #MPC.setTumbllerPath([0.6*np.cos(np.pi*t/10) for t in TBref_time], [0.6*np.sin(np.pi*t/10) for t in TBref_time], TBref_time) # reference is a dictionary
    #MPC.setTumbllerPath([0.075*t for t in TBref_time], [0.09*t for t in TBref_time], TBref_time) # reference is a dictionary
    #MPC.setTumbllerPath([-1+0.125*(t) for t in TBref_time], [0.01 for t in TBref_time], TBref_time)
    #MPC.setTumbllerPath([0.6*np.cos(0.25*t) for t in TBref_time], [0.3*np.sin(0.5*t) for t in TBref_time], TBref_time)
    #return MPC
    return FSM
    '''
    PID = PIDWrapper()
    TBref_time = [0.01*i for i in range(6000)]
    #PID.setTumbllerPath([0.075*t for t in TBref_time], [0.09*t for t in TBref_time], TBref_time)
    #PID.setTumbllerPath([0.6*np.cos(np.pi*t/10) for t in TBref_time], [0.6*np.sin(np.pi*t/10) for t in TBref_time], TBref_time)
    PID.setTumbllerPath([1.6 for t in TBref_time], [-1.2 for t in TBref_time], TBref_time)
    return PID
    '''
    


# Initialize Node
rospy.init_node("controller_node")
startTime = rospy.Time.now()

# Initialize Controller 
Controller = InitializeController()
#FSM = InitializeController() # RENAME

# CrazyFlie Communication

CF_CtrlPublisher = rospy.Publisher('/CF_Ctrl',CtrlCmd, queue_size=64)
print("Sending CrazyFlie Controller Commands")

CF_FbkListener = rospy.Subscriber('/CF_State_Feedback',StateFbk, process_CFfbk, (CF_CtrlPublisher, Controller, startTime))
print("Listening to CrazyFlie State Feedback")

# Tumbller Communication

TB_CtrlPublisher = rospy.Publisher('/TB_Ctrl',CtrlCmd, queue_size=64)
print("Sending Tumbller Controller Commands")

TB_FbkListener = rospy.Subscriber('/TB_State_Feedback',StateFbk, process_TBfbk, (Controller))
print("Listening to Tumbller State Feedback")

rate = rospy.Rate(100)
rospy.spin()