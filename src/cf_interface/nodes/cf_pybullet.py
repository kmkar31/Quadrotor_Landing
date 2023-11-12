#!/usr/bin/python3

import rospy
import pybullet as p
import pybullet_data
import rospkg
from msgs.msg import CtrlCmd, StateFbk
from std_msgs.msg import Header
import numpy as np

def init_bullet():
    # Load Environment
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    print("PyBullet Physics Environment Ready")
    
    
    startPos = [0,0,0.1]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    rp = rospkg.RosPack()
    # Load CrazyFlie
    path = rp.get_path("cf_interface") + "/model/CF.urdf"
    cf_id = p.loadURDF(path,startPos, startOrientation)
    print("CrazyFlie Loaded")

    return [cf_id, startPos]

def process_ctrl(msg, args):
    #print(msg.u)
    CF_ID = args[0]
    state = args[1]
    A = np.array(rospy.get_param("/MPCParams/A"))
    B = np.array(rospy.get_param("/MPCParams/B"))
    C = np.array(rospy.get_param("/MPCParams/C"))
    newstate = C@(A@np.linalg.pinv(C)@state + B@np.array(msg.u[0:3]).T)
    #print(newstate)
    #print(msg.u)
    #p.resetBasePositionAndOrientation(CF_ID,newstate,[0,0,0,0])
    p.resetBaseVelocity(CF_ID, msg.u[0:3])

def publish_feedback(CF_ID, pub):
    msg = StateFbk()
    msg.header = Header(stamp=rospy.Time.now(), frame_id="CF_state")
    msg.state = p.getBasePositionAndOrientation(CF_ID)[0] #+ 0.1*(-1 + 2*np.random.random((3,)))
    #print(msg.state)

    msg.name = "CrazyFlie State Feedback"
    pub.publish(msg)
    return msg.state
    

[CF_ID, state] = init_bullet()

rospy.init_node("cf_bullet_node")

CtrlListener = rospy.Subscriber('/CF_Ctrl',CtrlCmd, process_ctrl, (CF_ID, state))
print("Listening to Controller Commands")

FbkPublisher = rospy.Publisher('/CF_State_Feedback',StateFbk, queue_size=64)
print("Sending CF State Feedback")

p.setRealTimeSimulation(0)

feedbackrate = rospy.Rate(100)

while not(rospy.is_shutdown()):
    state = publish_feedback(CF_ID, FbkPublisher)
    p.stepSimulation()
    feedbackrate.sleep()
