import os
import sys
from math import pi
import Sofa.ImGui as MyGui
import Sofa.Core

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../../assets/")
import parts.connection.emiomotors as EmioMotors
from Serial import *
from CameraTargetController import *
from FlexTargetController import *
from enum import Enum

class MotorController(Sofa.Core.Controller):
    """
    Controller class for the Emio robot's motors.
    Send the angles to the motors.
    Only two possible configurations of the robot are considered in this lab.
    """
    
    def __init__(self, emio, config1_angles, config2_angles, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "MotorController"
        self.emio = emio
        #self.tilt = emio.getRoot().Simulation.tilt
        self.config1_angles = config1_angles
        self.config2_angles = config2_angles
        EmioMotors.openAndConfig()
        self.angles = [emio.getRoot().Simulation.tilt0, emio.getRoot().Simulation.tilt1, emio.getRoot().Simulation.tilt2, emio.getRoot().Simulation.tilt3]

    def onAnimateEndEvent(self, _):
        # Check and apply the chosen configuration 
        #angles = self.config1_angles
        #if self.tilt.value: 
            #angles = self.config2_angles
        
        # Apply the configuration to the motors in the simulation
        for i in range(4):
            self.emio.motors[i].getMechanicalState().rest_position.value = [[self.angles[i].value]]

        # Send the angles to the real robot
        if MyGui.getRobotConnection():
            EmioMotors.setAngle(self.angles)


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
    SensingMethod = Enum('Method', [('Camera', 1), ('Flex', 2)])
    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.VisualStyle.displayFlags = ["showVisual", "showInteractionForceFields"]

    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]


    # Add Emio to the scene
    emio = Emio(name="Emio",
                legsName=["blueleg"],
                legsModel=["beam"],
                legsPositionOnMotor=["counterclockwiseup", "clockwiseup", "counterclockwiseup", "clockwiseup"],
                centerPartName="yellowpart",
                centerPartType="rigid",
                extended=False)
    if not emio.isValid():
        return

    simulation.addChild(emio)
    addSolvers(emio, rayleighMass=0, rayleighStiffness=0)

    emio.attachCenterPartToLegs()
    assemblycontroller = AssemblyController(emio)
    emio.addObject(assemblycontroller)

    # In this lab we fix the motor
    for i in range(4):
        emio.motors[i].addObject("RestShapeSpringsForceField", points=[0], stiffness=1e9)

    # Let's now add the components for the connection to the real robot 
    dotTracker = None
    if args.connection:

        # Motors controller and configuration
        simulation.addData(name="tilt0", type="float", value=-0.48)
        simulation.addData(name="tilt1", type="float", value=0.48)
        simulation.addData(name="tilt2", type="float", value=-0.48)
        simulation.addData(name="tilt3", type="float", value=0.48)
        simulation.addData(name="Camerasensing", type = "bool", value = False)
        simulation.addData(name="TargetAngle1", type = "int", value = 0)
        simulation.addData(name="TargetAngle2", type = "int", value = 0)
        simulation.addData(name="TargetAngle3", type = "int", value = 0)
        simulation.addData(name="TargetAngle4", type = "int", value = 0)
        simulation.addData(name="TargetAngleAll", type = "int", value = 0)
        # Let's consider two configurations of the robot:
        # 1. The robot is in a upward straight configuration
        config1_angles = [-0.48, 0.48, -0.48, 0.48]
        # 2. The robot is tilted, in the direction away from the camera allowing it to better see the marker
        config2_angles = [-0.99, 0.99, 0.45, -0.45]
        rootnode.addObject(MotorController(emio, config1_angles, config2_angles))

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
                                                       filter_alpha=0.6)) # Factor used in the filter
                                                       #rotation=emio.Camera.torealrotation,
                                                       #translation=emio.Camera.torealtranslation))
        except RuntimeError as e:
            Sofa.msg_error(__file__, "Problem with the camera: " + str(e))

    # Add the position we want to observe (the effector)
    # emio.effector.addObject("MechanicalObject", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]])
    # emio.effector.addObject('PositionEffector', template="Rigid3", 
    #                         indices=[0],
    #                         useDirections=[0, 0, 0, 0, 0, 1],
    #                         limitShiftToTarget=True,
    #                         maxShiftToTarget=50,  # mm
    #                         effectorGoal=[0, 165, 0, 0, 0, 0, 1], 
    #                         name="CameraEffectorCoord")
    # emio.effector.addObject("RigidMapping", index=0)

    emio.addObject(serialReader())

    
    for leg in range(len(emio.legs)) :
        defPart = emio.legs[leg].getChild("Leg"+str(leg)+"DeformablePart")
        defPart.Leg.addObject('PositionEffector', template="Rigid3", 
                                    indices=[8],
                                    useDirections=[0, 0, 0, 1, 0, 1],
                                    limitShiftToTarget=True,
                                    maxShiftToTarget=50,  # mm
                                    effectorGoal=[0, 0, 0, 0, 0, 0, 1], 
                                    name="Flex" + str(leg) +"EffectorCoord")
        
    

    # Sensor (to retrieve the forces applied on the real robot effector)
    sensor = emio.centerpart.addChild("Sensor")
    sensor.addObject("MechanicalObject", position=[[0, 0, 0]])
    sensor.addObject("ForcePointActuator", 
                    name="ForcePointActuator",
                    indices=[0],
                    direction=[0, 0, 0],
                    applyForce=False, # We only want to apply the force when the assembly is done
                    showForce=True, 
                    maxForceVariation = 5,
                    visuScale=1,
                    )
    sensor.addObject("RigidMapping", index=0)
    sensor.addData(name="Force", type="Vec3d", value=[0, 0, 0]) # We use this to store and send the sensed force through ROS2

    # GUI
    MyGui.SimulationState.addData("Sensor", "Force", sensor.Force) # We use this to send the force through ROS2
    #.SimulationState.addData("TCP", "Frame", emio.effector.getMechanicalState().position) # We use this to send the position effector through ROS2
    MyGui.MyRobotWindow.addSetting("Configuration0", simulation.tilt0, -3.14, 3.14) # Add a setting ti the GUI to choose the configuration of the robot
    MyGui.MyRobotWindow.addSetting("Configuration1", simulation.tilt1, -3.14, 3.14)
    MyGui.MyRobotWindow.addSetting("Configuration2", simulation.tilt2, -3.14, 3.14)
    MyGui.MyRobotWindow.addSetting("Configuration3", simulation.tilt3, -3.14, 3.14)
    MyGui.MyRobotWindow.addSetting("Sensing Method", simulation.Camerasensing, 0, 1)
    MyGui.MyRobotWindow.addSetting("Angle target 1", simulation.TargetAngle1, -90, 15)
    MyGui.MyRobotWindow.addSetting("Angle target 2", simulation.TargetAngle2, -90, 15)
    MyGui.MyRobotWindow.addSetting("Angle target 3", simulation.TargetAngle3, -90, 15)
    MyGui.MyRobotWindow.addSetting("Angle target 4", simulation.TargetAngle4, -90, 15)
    MyGui.MyRobotWindow.addSetting("Angle target All", simulation.TargetAngleAll, -90, 15)


    if dotTracker is not None:
        # Feed the effector target with the camera position
        rootnode.addObject(CameraTargetController(rootnode=rootnode, 
                                            assemblycontroller=assemblycontroller, 
                                            sensor=sensor, isActive = simulation.Camerasensing))
    rootnode.addObject(FlexTargetController(rootnode=rootnode, 
                                            assemblycontroller=assemblycontroller, 
                                            sensor=sensor, isActive = simulation.Camerasensing, useSensor = [1, 1, 1, 1], serial = emio.serialReader))

    return rootnode
