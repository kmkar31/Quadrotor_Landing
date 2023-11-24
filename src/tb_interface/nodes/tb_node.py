#!/usr/bin/python3

import rospy
from msgs.msg import StateFbk, CtrlCmd
from std_msgs.msg import Header
import numpy as np
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cf_interface.CF import CF
import os



def send_fbk(publisher, state, filename, t):
    msg = StateFbk()
    msg.header = Header(stamp=rospy.Time.now(), frame_id="TB_state")
    msg.state = state
    msg.name = "Tumbller State Feedback"
    publisher.publish(msg)
    '''
    f = open(filename, 'a+')
    if os.path.getsize(filename) == 0:
        f.write("Time," + "X,Y,Z,Vx,Vy,Vz,r,p,y," + "\n")
    f.write(str(t) + ",")
    for x in state:
        f.write(str(x)+",")
    f.write("\n")
    f.close()
    '''

def process_ctrl(msg):
    #print("TB Control Commands", msg.u)
    pass

# Initialize Node
rospy.init_node("cf_node")

#CtrlListener = rospy.Subscriber('/TB_Ctrl',CtrlCmd, process_ctrl)
#print("Listening to Controller Commands")

FbkPublisher = rospy.Publisher('/TB_State_Feedback',StateFbk, queue_size=64)
print("Sending CF State Feedback")

rate = rospy.Rate(rospy.get_param('/ControlFrequency',50)*2)
uri = 'radio://1/80/250K/E7E7E7E7E7'
cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    CF = CF(scf.cf)

    FbkPublisher = rospy.Publisher('/TB_State_Feedback',StateFbk, queue_size=64)
    print("Sending Tumbller State Feedback")

    startTIme = rospy.Time.now()
    CF.log_start()
    filename = "../" + rospy.get_param("/LogDir") + "/" + str(rospy.Time.now()) + ".csv"
    while not(rospy.is_shutdown()):
        t = (rospy.Time.now()-startTIme).to_sec()
        send_fbk(FbkPublisher, CF.state, filename, t)
        rate.sleep()
    CF.log_stop()

rospy.spin()
    
    