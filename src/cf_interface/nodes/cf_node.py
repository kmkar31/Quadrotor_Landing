#!/usr/bin/python3

import rospy
#import cfclient
from msgs.msg import StateFbk, CtrlCmd
from std_msgs.msg import Header
import numpy as np
from cf_interface.CF import CF
import time
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.commander import Commander
from cflib.crazyflie.high_level_commander import HighLevelCommander

def send_fbk(publisher, state):
    if state is not None:
        msg = StateFbk()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="CF_state")
        msg.state = state
        msg.name = "CrazyFlie State Feedback"
        publisher.publish(msg)

def process_ctrl(msg, args):
    cf = args[0]
    hlcommander = args[1]
    u = msg.u
    #print(u)
    if msg.u[2] < -5:
        #hlcommander.land(0,1)
        cf.commander.send_notify_setpoint_stop()
        time.sleep(10)
    else:
        #cf.commander.send_velocity_world_setpoint(0,0,0,0)
        cf.commander.send_velocity_world_setpoint(u[0], u[1], u[2], 0)
        time.sleep(1/rospy.get_param('/ControlFrequency',0.02))
    cf.commander.send_notify_setpoint_stop()


# Initialize Node
rospy.init_node("cf_node")
rate = rospy.Rate(rospy.get_param('/ControlFrequency'))

uri = 'radio://0/100/2M/E7E7E7E7E7'
cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    print("Running")
    CF = CF(scf.cf)
    commander = Commander(scf.cf)
    hlcommander = HighLevelCommander(scf.cf)

    hlcommander.takeoff(0.5,2)
    time.sleep(2)
    hlcommander.go_to(0,0,0.55,0,3)
    time.sleep(3)
    CtrlListener = rospy.Subscriber('/CF_Ctrl',CtrlCmd, process_ctrl, (scf.cf, hlcommander))
    print("Listening to Controller Commands")

    FbkPublisher = rospy.Publisher('/CF_State_Feedback',StateFbk, queue_size=64)
    print("Sending CF State Feedback")

    CF.log_start()
    while not(rospy.is_shutdown()):
        send_fbk(FbkPublisher, CF.state)
        rate.sleep()
    
    hlcommander.land(0,2)
    time.sleep(2)
    CF.log_stop()

rospy.spin()
    
    