import os
import sys
from math import pi
import Sofa.ImGui as MyGui
import Sofa.Core

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../EmioLabs/assets/")
import parts.connection.emiomotors as EmioMotors

NBEMIOS = 2


class MotorController(Sofa.Core.Controller):
    """
    Controller class for the Emio robot's motors.
    Send the angles to the motors.
    Only two possible configurations of the robot are considered in this lab.
    """
    
    def __init__(self, emios, config1_angles, config2_angles, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "MotorController"
        self.emios = emios
        self.tilt = emios[0].getRoot().Simulation.tilt
        self.config1_angles = config1_angles
        self.config2_angles = config2_angles
        EmioMotors.openAndConfig()

    def onAnimateEndEvent(self, _):
        # Check and apply the chosen configuration 
        angles = self.config1_angles
        if self.tilt.value: # EmioSensor
            angles = self.config2_angles
        
        # Apply the configuration to the motors in the simulation
        for emio in self.emios:
            for i in range(4):
                emio.motors[i].getMechanicalState().rest_position.value = [[angles[i]]]

        # Send the angles to the real robot
        if MyGui.getRobotConnection():
            EmioMotors.setAngle(angles)

class TargetController(Sofa.Core.Controller):
    """
    Controller class to set the target position of the effector.
    The target is the effector position of the real device given by the depth camera.
    To correct the error from the position of the camera, we substract the error calculated at the end of the assembly animation. 
    """
    def __init__(self, rootnode, assemblycontroller, sensor):
        Sofa.Core.Controller.__init__(self)
        self.name = "TargetController"
        self.rootnode = rootnode
        self.target = rootnode.DepthCamera.getMechanicalState()
        self.effector1 = rootnode.Simulation.EmioSensor.CenterPart.Effector
        self.effector2 = rootnode.Simulation.EmioPhantom.CenterPart.Effector
        self.assemblycontroller = assemblycontroller
        self.sensor = sensor

        # The camera error correction
        self.correctiondone = False
        self.cameracorrection = [0, 0, 0]

    def onAnimateBeginEvent(self, _):
        
        # When the assembly animation is done, we can correct the camera position
        # Once the correction calculated, we can apply the sensed force to the simulated robot
        if not self.correctiondone and self.assemblycontroller.done:
            self.correctiondone = True # We only want to do this once
            for i in range(NBEMIOS):
                self.cameracorrection[i] = self.target.position.value[0][i] - self.effector1.getMechanicalState().position.value[0][i]
            self.rootnode.Simulation.EmioPhantom.CenterPart.Sensor.ForcePointActuator.applyForce=True

        # Validate the force applied on the sensor
        # f = self.rootnode.Simulation.EmioSensor.CenterPart.Sensor.ForcePointActuator.force.value
        # dt = self.rootnode.dt.value
        # s = 1./dt # The value from ForcePointActuator is an impulse and should be divide by the time step to retreive the force
        # s *= 0.055 # TODO: weird factor !!! 
        # self.rootnode.Simulation.EmioValidation.CenterPart.Effector.ConstantForceField.totalForce.value = [f[0]*s, f[1]*s, f[2]*s]

        # Update the target position of the effector
        # The target position is the position from the camera minus the correction
        for effector in [self.effector1, self.effector2]:
            effector.EffectorCoord.effectorGoal.value = [[self.target.position.value[0][0] - self.cameracorrection[0],
                                                           self.target.position.value[0][1] - self.cameracorrection[1],
                                                           self.target.position.value[0][2] - self.cameracorrection[2]]]

def getParserArgs():
    """
    Parse the command line arguments.
    :return: args
    """
    import argparse

    parser = argparse.ArgumentParser(prog=sys.argv[0], description='Simulate the robot Emio.')
    parser.add_argument('--no-connection', dest="connection", action='store_false',
                        help="use when you want to run the simulation without the robot")

    try:
        args = parser.parse_args()
    except SystemExit:
        Sofa.msg_error(sys.argv[0], "Invalid arguments, get defaults instead.")
        args = parser.parse_args([])
    return args

def createScene(rootnode):
    from utils.header import addHeader, addSolvers
    from parts.controllers.assemblycontroller import AssemblyController
    from parts.emio import Emio

    args = getParserArgs()

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    addSolvers(simulation, rayleighMass=0, rayleighStiffness=0)

    emios = [] # Let's create three Emios, they will be used as a sensor, a phantom, and the last one for validation
    for i in range(NBEMIOS):
        # Add Emio to the scene
        emio = Emio(name="Emio" + ["Sensor", "Phantom", "Validation"][i],
                    legsName=["blueleg"],
                    legsModel=["beam"],
                    legsPositionOnMotor=["clockwiseup", "counterclockwiseup", "clockwiseup", "counterclockwiseup"],
                    centerPartName="yellowpart",
                    centerPartType="rigid",
                    extended=False)
        if not emio.isValid():
            return
        emios.append(emio)

        simulation.addChild(emio)
        emio.attachCenterPartToLegs()
        assemblycontroller = AssemblyController(emio)
        emio.addObject(assemblycontroller)

        # In this lab we fix the motor
        for i in range(4):
            emio.motors[i].addObject("RestShapeSpringsForceField", points=[0], stiffness=1e6)

    # Let's now add the components for the connection to the real robot 
    dotTracker = None
    if args.connection:

        # Motors controller and configuration
        simulation.addData(name="tilt", type="bool", value=False)
        # Let's consider two configurations of the robot:
        # 1. The robot is in a upward straight configuration
        config1_angles = [1., -1., 1., -1.]
        # 2. The robot is tilted, in the direction away from the camera allowing it to better see the marker
        config2_angles = [1.62, -1.62, 0.66, -0.66]
        rootnode.addObject(MotorController(emios, config1_angles, config2_angles))

        # Camera
        try:
            from parts.controllers.trackercontroller import DotTracker
            dotTracker = rootnode.addObject(DotTracker(name="DotTracker",
                                                       root=rootnode,
                                                       nb_tracker=1,
                                                       show_video_feed=False,
                                                       track_colors=True,
                                                       comp_point_cloud=False,
                                                       scale=1,
                                                       filter_alpha=0.9, # Factor used in the filter
                                                       rotation=simulation.EmioSensor.Camera.torealrotation,
                                                       translation=simulation.EmioSensor.Camera.torealtranslation))
        except RuntimeError:
            Sofa.msg_error(__file__, "Camera not detected")

    if dotTracker is not None:
        for emio in [simulation.EmioSensor, simulation.EmioPhantom]:
            # Add the position we want to observe (the effector)
            emio.effector.addObject("MechanicalObject", position=[[0, 0, 0]])
            emio.effector.addObject('PositionEffector',
                                    indices=[0],
                                    useDirections=[1, 1, 1, 0, 0, 0],
                                    limitShiftToTarget=True,
                                    maxShiftToTarget=50,  # mm
                                    effectorGoal=[0,182,0], 
                                    name="EffectorCoord")
            emio.effector.addObject("RigidMapping", index=0)

            # Sensor (to retrieve the forces applied on the real robot effector)
            sensor = emio.centerpart.addChild("Sensor")
            sensor.addObject("MechanicalObject", position=[[0, 0, 0]])
            sensor.addObject("ForcePointActuator", 
                            name="ForcePointActuator",
                            indices=[0],
                            direction=[0, 0, 0],
                            applyForce=False, # we won't apply the force on the Sensor model, instead we apply the force on the other Emios for phantom and validation
                            showForce=False
                            )
            sensor.addObject("RigidMapping", index=0)

        simulation.EmioPhantom.addObject("VisualStyle", displayFlags=["showVisualModels"])
        simulation.EmioSensor.addObject("VisualStyle", displayFlags=["hideVisualModels"])

        # The Emio used for validation
        # simulation.EmioValidation.addObject("VisualStyle", displayFlags=["showVisualModels"])
        # simulation.EmioValidation.effector.addObject("MechanicalObject", position=[[0, 0, 0]])
        # simulation.EmioValidation.effector.addObject("ConstantForceField", indices=[0], totalForce=[0., 0., 0.], showArrowSize=0.02) # we apply the sensed force here
        # simulation.EmioValidation.effector.addObject("VisualStyle", displayFlags=["showForceFields"])
        # simulation.EmioValidation.effector.addObject("RigidMapping", index=0)

        # Feed the effector target with the camera position
        rootnode.addObject(TargetController(rootnode=rootnode, 
                                            assemblycontroller=assemblycontroller, 
                                            sensor=sensor))

        # GUI
        MyGui.SimulationState.addData("Sensor", "Force", simulation.EmioSensor.effector.EffectorCoord.effectorGoal) # We use this to send the force through ROS2
        MyGui.MyRobotWindow.addSetting("Configuration", simulation.tilt, 0, 1) # Add a setting ti the GUI to choose the configuration of the robot

    return rootnode
