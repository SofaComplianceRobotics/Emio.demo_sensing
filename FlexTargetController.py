import os
import sys
from math import pi
import Sofa.Core
from scipy.spatial.transform import Rotation as R
import numpy as np

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../../assets/")

from Serial import *

class FlexTargetController(Sofa.Core.Controller):
    def __init__(self, rootnode, assemblycontroller, sensor, isActive, useSensor = [1, 1, 1, 1], serial = None):
        Sofa.Core.Controller.__init__(self)
        self.name = "FlexTargetController"
        self.rootnode = rootnode
        self.serial = serial
        self.useSensor = useSensor
        hierarchy = rootnode.Simulation.Emio
        self.effector = [hierarchy.Leg0.Leg0DeformablePart.Leg.Flex0EffectorCoord, 
                        hierarchy.Leg1.Leg1DeformablePart.Leg.Flex1EffectorCoord,
                        hierarchy.Leg2.Leg2DeformablePart.Leg.Flex2EffectorCoord,
                        hierarchy.Leg3.Leg3DeformablePart.Leg.Flex3EffectorCoord]
        self.assemblycontroller = assemblycontroller
        self.sensor = sensor
        self.isActive = isActive
        self.serial.Start()


    def onAnimateBeginEvent(self, _):
        if not(self.isActive.value) :
            if self.assemblycontroller.done:
                self.rootnode.Simulation.Emio.CenterPart.Sensor.ForcePointActuator.applyForce=True
                

            # Update the target position of the effector
            if self.serial != None :
                for leg in range(4):
                    if self.useSensor[leg] == 1:
                        sensorData = self.serial.getAngles(leg)
                        if leg == 0:
                            goalOrientation = (R.from_euler('xyz', [[-130.7 - self.rootnode.Simulation.TargetAngle1.value - self.rootnode.Simulation.TargetAngleAll.value, 0, -90]], degrees=True)).as_quat()
                        if leg == 1:
                            goalOrientation = (R.from_euler('xyz', [[-130.7 - self.rootnode.Simulation.TargetAngle2.value - self.rootnode.Simulation.TargetAngleAll.value, 0, -90]], degrees=True)).as_quat()
                        if leg == 2:
                            goalOrientation = (R.from_euler('xyz', [[-130.7 - self.rootnode.Simulation.TargetAngle3.value - self.rootnode.Simulation.TargetAngleAll.value, 0, -90]], degrees=True)).as_quat()
                        if leg == 3:
                            goalOrientation = (R.from_euler('xyz', [[-130.7 - self.rootnode.Simulation.TargetAngle4.value - self.rootnode.Simulation.TargetAngleAll.value, 0, -90]], degrees=True)).as_quat()
                        # print(goalOrientation[0][0])
                        # print(goalOrientation[0][1])
                        # print(goalOrientation[0][2])
                        # print(goalOrientation[0][3])
                        self.effector[leg].effectorGoal.value = [[0,0,0,
                                                                            goalOrientation[0][0], 
                                                                            -goalOrientation[0][1], 
                                                                            -goalOrientation[0][2], 
                                                                            -goalOrientation[0][3]]]
        
    def onAnimateEndEvent(self, _):
        if not(self.isActive.value) :
            # Update the force applied on the sensor
            dt = self.rootnode.dt.value
            self.sensor.Force.value = [self.sensor.ForcePointActuator.force.value[0] / dt,
                                    self.sensor.ForcePointActuator.force.value[1] / dt,
                                    self.sensor.ForcePointActuator.force.value[2] / dt]
            