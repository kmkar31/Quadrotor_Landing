#!/usr/bin/python3

import rospy
import numpy as np
import os
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
        self.SearchErrTol = rospy.get_param('/SearchErrTol', 0.05)
        self.LandErrTol = rospy.get_param('/LandErrTol', 0.05)
        self.freq = rospy.get_param('/ControlFrequency',50)
        self.EndTime = rospy.get_param('/EndTime',60)
        self.altitude = 0.55
        self.InterpDegree = 1 # Set Either 1 or 2
        self.PrevTime = 0

        self.filename = "../" + rospy.get_param("/LogDir") + "/" + str(rospy.Time.now()) + ".csv"
        
        if self.TrackMode == 1: # Track Real Tumbller
            self.Nstate = rospy.get_param('/Nstate',6)
            self.TumbllerHistory = []

        elif self.TrackMode == 2:
            if TumbllerPath is None:
                raise Exception("Virtual Tracking Commanded but Tumbller path is not provided")
            self.TumbllerPath = TumbllerPath
        
        if self.ControlMode == 1: # MPC
            self.MPCInit()
        elif self.ControlMode == 2: # PID
            self.PIDInit()
        else : 
            raise Exception("Undefined Control Method")
    
    def MPCInit(self):
        params = rospy.get_param("/MPCParams")
        self.SearchController = MPCWrapper(params["m"], params["p"], params["q"], params["N"],
                     params["A"], params["B"], params["C"], params["Q"],
                     params["R"], params["ymin"], params["ymax"], 
                     params["umin"], params["umax"])
        self.LandController = MPCWrapper(params["m"], params["p"], params["q"], params["N"],
                     params["A"], params["B"], params["C"], params["Qland"],
                     params["Rland"], params["yminland"], params["ymax"], 
                     params["uminland"], params["umax"])
        
        if self.TrackMode == 2:
            self.SearchController.setReference(self.TumbllerPath[0], self.TumbllerPath[1], [0.55 for i in range(len(self.TumbllerPath[0]))],self.TumbllerPath[2])
            self.LandController.setReference(self.TumbllerPath[0], self.TumbllerPath[1], [0.55 for i in range(len(self.TumbllerPath[0]))],self.TumbllerPath[2])

        self.Controller = self.SearchController
    
    def PIDInit(self):
        self.Controller = PIDWrapper()
    
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CONTROL MOVES  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def NextMove(self,t,y0):
        self.altitude = y0[2]
        if self.State == "search":
            if len(self.Controller.RefStore) == 0:
                return [0,0,0.3]
            u = self.Controller.NextMove(t,y0) + [0,0,0.3]
            e = self.Controller.e
            if np.linalg.norm(e[0:2]) <= self.SearchErrTol:
                self.StateTransition("land", t, np.reshape(y0,(-1)))
        
        elif self.State == "land":
            u = self.Controller.NextMove(t,y0)
            #e = self.Controller.e
            if np.linalg.norm(y0[2]) <= self.LandErrTol or t >= self.EndTime:
                self.StateTransition("AllStop")
        

        elif self.State == "AllStop":
            u = [0,0,-100]

        if hasattr(self.Controller, "ref"):
            self.log(t,np.reshape(y0,(-1)),np.reshape(self.Controller.ref[0:3,:],(-1)), u)
        return u
    
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ AUXILIARY FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def StateTransition(self,newState, t=None, y0=None):
        if newState not in self.StateList:
            raise Exception("Unknown State Commanded from State Machine")
        self.State = newState
        print("Switched to ",self.State, " phase")
        if self.State == 'land':
            ref = self.Controller.Reference
            self.Controller = self.LandController
            if self.TrackMode == 1:
                #self.setReference(y0,t)
                self.Controller.RefStore = ref
                pass
            elif self.TrackMode == 2:
                self.Controller.setReference(self.TumbllerPath[0], self.TumbllerPath[1], 
                                         [0.5*(1-(i/len(self.TumbllerPath[0]))) for i in range(len(self.TumbllerPath[0]))],self.TumbllerPath[2])

    def setReference(self, state, t):
        # If TrackMode  = 1, As and when you receive a reference from the Tumbller, update the variable
        #if t-self.PrevTime>1:
        '''
        self.TumbllerHistory.append([state[0], state[1], state[2], t])
            #self.PrevTime = t
        if len(self.TumbllerHistory)>self.Nstate:
            self.TumbllerHistory.pop(0)
        if self.TrackMode == 1:
            timevec = np.linspace(t,self.EndTime,num=(1 + int((self.EndTime-t)*self.freq)))
            #x = [state[0] for i in range(len(timevec))]
            #y = [state[1] for i in range(len(timevec))]
            if len(self.TumbllerHistory)<self.Nstate:
                x = [state[0] for i in range(len(timevec))]
                y = [state[1] for i in range(len(timevec))]
            else:
                x_vals = [point[0] for point in self.TumbllerHistory]
                y_vals = [point[1] for point in self.TumbllerHistory]
                #z_vals = np.array([point[2] for point in self.TumbllerHistory])
                time_vals = [point[3] for point in self.TumbllerHistory]

                px = np.polyfit(time_vals, x_vals, self.InterpDegree)
                py = np.polyfit(time_vals, y_vals, self.InterpDegree)
                #pz = np.polyfit(time_vals, z_vals, self.InterpDegree)

                x = np.polyval(px, timevec)
                y = np.polyval(py, timevec)

                #z = np.polyval(pz, timevec)
            if self.State == "land":
                z = [(state[2] + (self.altitude-state[2])*(1 - i/len(timevec))) for i in range(len(timevec))]
            else:
                z = [0.55 for i in range(len(timevec))]
            self.Controller.setReference(x,y,z,timevec)
        else:
            self.Controller.setReference(state[0], state[1], state[2], t)
        '''
        #print("FSM", state)
        timevec = np.linspace(t,self.EndTime,num=(1 + int((self.EndTime-t)*self.freq)))
        x = [state[0] for i in range(len(timevec))]
        y = [state[1] for i in range(len(timevec))]
        if self.State == "land":
            z = [(state[2] + (self.altitude-state[2])*(1 - i/len(timevec))) for i in range(len(timevec))]
        else:
            z = [0.55 for i in range(len(timevec))]
        self.Controller.setReference(x,y,z,timevec)
        #'''
        


    def log(self, t, state, ref, u):
        f = open(self.filename, 'a+')
        if os.path.getsize(self.filename) == 0:
            f.write("Time," + "X,Y,Z," + "rx,ry,rz," + "Ux,Uy,Uz"+"\n")
        f.write(str(t) + ",")
        for x in state:
            f.write(str(x)+",")
        for r in ref:
            f.write(str(r)+",")
        for ui in u:
            f.write(str(ui)+",")
        f.write("\n")
        f.close()
