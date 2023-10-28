#!/usr/bin/python3

import rospy
#import cfclient
from msgs.msg import StateFbk, CtrlCmd
from std_msgs.msg import Header
import numpy as np
from src.CF import CF
import time
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.commander import Commander


def send_fbk(publisher, state):
    if state is not None:
        msg = StateFbk()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="CF_state")
        msg.state = state
        msg.name = "CrazyFlie State Feedback"
        publisher.publish(msg)

def process_ctrl(msg, args):
    #print("CF Control Commands", msg.u)
    print("Control Received")
    cf = args
    u = msg.u
    et = time.time() + 5
    while time.time() <= et:
        cf.commander.send_position_setpoint(0,0,1,0)
        time.sleep(0.02)


# Initialize Node
rospy.init_node("cf_node")
rate = rospy.Rate(100)

uri = 'radio://0/80/2M/E7E7E7E7E7'
cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    CF = CF(scf.cf)
    commander = Commander(scf.cf)
    CtrlListener = rospy.Subscriber('/CF_Ctrl',CtrlCmd, process_ctrl, scf.cf)
    print("Listening to Controller Commands")

    FbkPublisher = rospy.Publisher('/CF_State_Feedback',StateFbk, queue_size=64)
    print("Sending CF State Feedback")

    CF.log_start()
    while not(rospy.is_shutdown()):
        send_fbk(FbkPublisher, CF.state)
        rate.sleep()
    CF.log_stop()

rospy.spin()
    
    