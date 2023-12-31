#!/usr/bin/python3

import rospy
import numpy as np
import numpy.matlib as mat
from cvxopt import solvers
from cvxopt import matrix
import os

class MPCWrapper():
    def __init__(self, m, p, q, N, A, B, C, Q, R, ymin, ymax, umin, umax) -> None:
        # m = no of states
        # p = no of inputs
        # q = no of outputs
        self.m = m
        self.p = p
        self.q = q
        self.N = N
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)
        self.Q = np.array(Q)
        self.R = np.array(R)
        self.umin = np.array(umin)
        self.umax = np.array(umax)
        self.ymin = np.array(ymin)
        self.ymax = np.array(ymax)
        self.ustore = []
        self.xstore = []
        self.precompute()
        self.StartTime = 0
       
        self.T = rospy.get_param('/EndTime',60)
        self.freq = rospy.get_param("/ControlFrequency", 50) # What Frequency are we controlling at
        self.Reference = dict()
        self.RefStore = dict()
    
    def setStartTime(self,t):
        self.StartTime = t

    def precompute(self):
        self.T = np.zeros((self.m*self.N, self.p*self.N))
        for i in range(self.N):
            if i==0:
                self.S = self.A
            else:
                self.S = np.vstack((self.S, np.linalg.matrix_power(self.A, i+1)))
            for j in range(self.N):
                self.T[self.m*j:self.m*(j+1), self.p*i:self.p*(i+1)] = np.linalg.matrix_power(self.A,j)@self.B
        self.Cbar = np.array(np.kron(np.eye(self.N),self.C))
        self.Qbar = np.array(np.kron(np.eye(self.N),self.Q))
        self.Rbar = np.array(np.kron(np.eye(self.N),self.R))
        self.QuadProgSetup()
    
    def QuadProgSetup(self):
        # Objective Function Matrices
        self.H = 2*(self.T.T@self.Cbar.T@self.Qbar@self.Cbar@self.T + self.Rbar)
        self.H = (self.H + self.H.T)/2
        self.Fx = 2*self.T.T@self.Cbar.T@self.Qbar@self.Cbar@self.S
        self.Fr = 2*self.T.T@self.Cbar.T@self.Qbar

        # Constraint Matrices
        self.G = np.vstack((np.eye((self.N*self.p)), -np.eye((self.N*self.p)), self.Cbar@self.T, -self.Cbar@self.T))
        self.W = lambda x0 : np.vstack((mat.repmat(self.umax,self.N,1), 
                                        mat.repmat(-self.umin, self.N, 1),
                                        mat.repmat(self.ymax,self.N,1) - self.Cbar@self.S@x0,
                                        mat.repmat(-self.ymin, self.N, 1) + self.Cbar@self.S@x0))
  
    def setReference(self, x,y,z, timevec):
        # Creates a dictionary of reference paths at corresponding times
        RefStore = dict()
        RefStore['x'] = np.reshape(x,(-1,))
        RefStore['y'] = np.reshape(y,(-1,))
        RefStore['z'] = np.reshape(z,(-1,))
        RefStore['time'] = timevec
        self.RefStore = RefStore
    
    def NextMove(self, t, y0):
        # Build the Reference Signal
        self.Reference = self.RefStore
        #print(len(self.Reference['x']), len(self.Reference['time']))
        horizonTimes = [t + i/self.freq for i in range(self.N)]
        # Interpolates the reference values at the control timesteps
        xref = np.interp(horizonTimes, self.Reference['time'], self.Reference['x'])
        yref = np.interp(horizonTimes, self.Reference['time'], self.Reference['y'])
        zref = np.interp(horizonTimes, self.Reference['time'], self.Reference['z'])
        self.ref = []
        for i in range(len(horizonTimes)):            
            self.ref.append([[xref[i]],[yref[i]],[zref[i]]])
        self.ref = np.reshape(self.ref, (-1,1))

        self.e = np.reshape(y0[0:3,:] - self.ref[0:3,:],(-1))

        #print("MPC", self.ref[0:3])
        # Construct Quadprog:
        x0 = np.linalg.pinv(self.C)@y0[0:3] # Convert measurements back to state - Works because measurements are filtered
        f = self.Fx@x0 - self.Fr@self.ref
        solvers.options['show_progress'] = False
        sol = solvers.qp(matrix(self.H), matrix(f), matrix(self.G), matrix(np.array(self.W(x0))))
        z = np.array(sol['x'])
        if len(z)==0:
            raise Exception("Solution to QP Non-existent")
        self.u = np.reshape(z[0:self.q],(-1,))
        if self.u[2] > self.umax[2]:
            self.u[2] = self.umax[2]
        return self.u


class PIDWrapper():
    def __init__(self) -> None:
        self.Kp = rospy.get_param("/PIDParams/Kp")
        self.Kd = rospy.get_param("/PIDParams/Kd")
        self.Ki = rospy.get_param("/PIDParams/Ki")
        
        self.ControlFrequency = rospy.get_param("/ControlFrequency")
        self.umin = rospy.get_param("/PIDParams/umin")
        self.umax = rospy.get_param("/PIDParams/umax")

        self.e = np.zeros((3,1))
        self.eprev = np.zeros((3,1))
        self.u = np.zeros((3,))
        self.eint = np.zeros((3,))

        self.filename = "../" + rospy.get_param("/LogDir") + "/" + str(rospy.Time.now()) + ".csv"
    

    def setTumbllerPath(self, x,y, timevec):
        # Creates a dictionary of reference paths at corresponding times
        self.refx = x
        self.refy = y
        self.refz = [0.3 for i in range(len(timevec))]
        self.ref_time = timevec


    def NextMove(self, t, state):
        #print(state)

        self.eprev = self.e
        ref = []
        x = np.interp(t, self.ref_time, self.refx)
        y = np.interp(t, self.ref_time, self.refy)
        z = np.interp(t, self.ref_time, self.refz)
        
        ref = [[x],[y],[z]]
        self.e = (ref - state[0:3])
        self.eint = self.eint + self.e/self.ControlFrequency
        ed = (self.eprev - self.e)*self.ControlFrequency

        self.u = np.reshape(self.Kp*self.e + self.Kd * ed,(-1,))
        for i in range(3):
            self.u[i] = np.clip(self.u[i], a_min=self.umin[i], a_max=self.umax[i])
        if np.linalg.norm(self.e[0:2])<=0.1:
            self.Ki = 0
            self.u = [0,0,-10]
        else:
            self.log(t,ref,state)
        self.u = self.u + [0,0,0.5]
        
        return self.u
    
    def log(self, t,ref, state):
        self.state = np.reshape(state, (-1))
        f = open(self.filename, 'a+')
        if os.path.getsize(self.filename) == 0:
            f.write("Time," + "X,Y,Z,Vx,Vy,Vz,r,p,y," + "rx,ry,rz," + "Ux,Uy,Uz"+"\n")
        f.write(str(t) + ",")
        for x in self.state:
            f.write(str(x)+",")
        for r in ref:
            f.write(str(r[0])+",")
        for u in self.u:
            f.write(str(u)+",")
        f.write("\n")
        f.close()
        
        
        



