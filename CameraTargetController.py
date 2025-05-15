import os
import sys
from math import pi
import Sofa.ImGui as MyGui
import Sofa.Core

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../../assets/")
import parts.connection.emiomotors as EmioMotors

class CameraTargetController(Sofa.Core.Controller):
    """
    Controller class to set the target position of the effector.
    The target is the effector position of the real device given by the depth camera.
    To correct the error from the position of the camera, we substract the error calculated at the end of the assembly animation. 
    """
    def __init__(self, rootnode, assemblycontroller, sensor, isActive):
        Sofa.Core.Controller.__init__(self)
        self.name = "CameraTargetController"
        self.rootnode = rootnode
        self.target = rootnode.DepthCamera.getMechanicalState()
        self.effector = rootnode.Simulation.Emio.CenterPart.Effector
        self.assemblycontroller = assemblycontroller
        self.sensor = sensor
        self.isActive = isActive

        # The camera error correction
        self.correctiondone = False
        self.cameracorrection = [0, 0, 0]

    def onAnimateBeginEvent(self, _):
        if self.isActive:
        # When the assembly animation is done, we can correct the camera position
        # Once the correction calculated, we can apply the sensed force to the simulated robot
            if not self.correctiondone and self.assemblycontroller.done:
                self.correctiondone = True # We only want to do this once
                for i in range(3):
                    self.cameracorrection[i] = self.target.position.value[0][i] - self.effector.getMechanicalState().position.value[0][i]
                self.rootnode.Simulation.Emio.CenterPart.Sensor.ForcePointActuator.applyForce=True

            # Update the target position of the effector
            # The target position is the position from the camera minus the correction
            self.effector.CameraEffectorCoord.effectorGoal.value = [[self.target.position.value[0][0] - self.cameracorrection[0],
                                                                self.target.position.value[0][1] - self.cameracorrection[1],
                                                                self.target.position.value[0][2] - self.cameracorrection[2],
                                                                0., 0., 0., 1.]]
        
    def onAnimateEndEvent(self, _):
        if self.isActive:
            # Update the force applied on the sensor
            dt = self.rootnode.dt.value
            self.sensor.Force.value = [self.sensor.ForcePointActuator.force.value[0] / dt,
                                    self.sensor.ForcePointActuator.force.value[1] / dt,
                                    self.sensor.ForcePointActuator.force.value[2] / dt]