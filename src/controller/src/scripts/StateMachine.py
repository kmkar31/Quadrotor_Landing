#!/usr/bin/python3

import rospy
import numpy as np
from scripts.CF_controller import MPCWrapper, PIDWrapper

class StateMachine():
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ INIT FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def __init__(self, TumbllerPath = None) -> None:
        self.State = "search" # STATE is one of "search" or "land" or "AllStop"
        self.StateList = ["search", "land", "AllStop"]
        self.ControlMode = rospy.get_param('/ControlMode')
        self.TrackMode = rospy.get_param('/TrackMode',2)
        if self.ControlMode == 1: # MPC
            self.MPCInit()
        elif self.ControlMode == 2: # PID
            self.PIDInit()
        else : 
            raise Exception("Undefined Control Method")
        
        if self.TrackMode == 1: # Track Real Tumbller
            self.Nstate = rospy.get_param('/Nstate',3)
            self.TumbllerHistory = np.zeros((self.Nstate,3))

        elif self.TrackMode == 2:
            if TumbllerPath is None:
                raise Exception("Virtual Tracking Commanded but Tumbller path is not provided")
            self.TumbllerPath = TumbllerPath
        
        
        self.errTol = rospy.get_param('/ErrTol', 0.05)
    
    def MPCInit(self):
        params = rospy.get_param("/MPCParams")
        self.SearchController = MPCWrapper(params["m"], params["p"], params["q"], params["N"],
                     params["A"], params["B"], params["C"], params["Q"],
                     params["R"], params["ymin"], params["ymax"], 
                     params["umin"], params["umax"])
        self.LandController = MPCWrapper(params["m"], params["p"], params["q"], params["N"],
                     params["A"], params["B"], params["C"], params["Qland"],
                     params["Rland"], params["ymin"], params["ymax"], 
                     params["umin"], params["umax"])
        self.Controller = self.SearchController
    
    def PIDInit(self):
        self.Controller = PIDWrapper()
    
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CONTROL MOVES  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def NextMove(self,t,y0):

        if self.State == "search":
            u = self.Controller.NextMove(t,y0)
            e = self.Controller.e
            if np.linalg.norm(e[0:2]) <= self.errTol:
                self.StateTransition("land")
        
        elif self.State == "land":
            u = self.Controller.NextMove(t,y0)
            e = self.Controller.e
            if np.linalg.norm(e) <= self.errTol:
                self.StateTransition("AllStop")
        
        elif self.State == "AllStop":
            u = [0,0,-100]

        return u
    
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ AUXILIARY FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def StateTransition(self,newState):
        if newState not in self.StateList:
            raise Exception("Unknown State Commanded from State Machine")
        self.State = newState
        if self.State == 'land':
            self.Controller = self.LandController

    def setReference(self):
        # If TrackMode  = 1, As and when you receive a reference from the Tumbller, update the variable
        pass

    def queryReference(self):
        # If trackMode = 2, query the trajectory and interpolate at times t:1/freq:t+N/freq to obtain an N-sample reference
        # If trackMode = 1, use the position history to construct a N-sample reference
        # If landing mode, also generate the minimum jerk trajectory over Z
        # If not landing mode, hold Z constant at 0.55m
        pass
