from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import logging
import time
import rospy

class CF():
    def __init__(self, cf):
        self.cf = cf
        logging.basicConfig(level=logging.ERROR)
        self.log_setup()
        self.state = []

    def log_setup(self):
        self.logcon = LogConfig(name="StateEstimate", period_in_ms=10)

        #self.logcon.add_variable('kalman.stateX')
        #self.logcon.add_variable('kalman.stateY')
        #self.logcon.add_variable('kalman.stateZ')
        self.logcon.add_variable('stateEstimateZ.x')
        self.logcon.add_variable('stateEstimateZ.y')
        self.logcon.add_variable('stateEstimateZ.z')
        self.logcon.add_variable('stateEstimateZ.vx')
        self.logcon.add_variable('stateEstimateZ.vy')
        self.logcon.add_variable('stateEstimateZ.vz')
        self.logcon.add_variable('stateEstimate.roll')
        self.logcon.add_variable('stateEstimate.pitch')
        self.logcon.add_variable('stateEstimate.yaw')
        
        self.cf.log.add_config(self.logcon)
        self.logcon.data_received_cb.add_callback(self.process_log)
        self.logcon.start()
    
    def log_start(self):
        self.logcon.start()
        print("Starting Log")
    
    def log_stop(self):
        print("Stopping Log")
        self.logcon.stop()
    
    def process_log(self, timestamp, data, logcon):
        self.state = []
        #self.state.append(data['kalman.stateX'])
        #self.state.append(data['kalman.stateY'])
        #self.state.append(data['kalman.stateZ'])
        
        self.state.append(data['stateEstimateZ.x']/1000.0) #mm to m
        self.state.append(data['stateEstimateZ.y']/1000.0)
        self.state.append(data['stateEstimateZ.z']/1000.0)
        
        self.state.append(data['stateEstimateZ.vx']/1000.0)
        self.state.append(data['stateEstimateZ.vy']/1000.0)
        self.state.append(data['stateEstimateZ.vz']/1000.0)

        self.state.append(round(data['stateEstimate.roll'],2))
        self.state.append(round(data['stateEstimate.pitch'],2))
        self.state.append(round(data['stateEstimate.yaw'],2))
