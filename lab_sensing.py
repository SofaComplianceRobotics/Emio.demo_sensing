import os
import sys
from math import pi
import Sofa.ImGui as MyGui
import Sofa.Core

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../EmioLabs/assets/")
import parts.connection.emiomotors as EmioMotors


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
        self.tilt = emio.getRoot().Simulation.tilt
        self.config1_angles = config1_angles
        self.config2_angles = config2_angles
        EmioMotors.openAndConfig()

    def onAnimateEndEvent(self, _):
        # Check and apply the chosen configuration 
        angles = self.config1_angles
        if self.tilt.value: 
            angles = self.config2_angles
        
        # Apply the configuration to the motors in the simulation
        for i in range(4):
            self.emio.motors[i].getMechanicalState().rest_position.value = [[angles[i]]]

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
        self.effector = rootnode.Simulation.Emio.CenterPart.Effector
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
            for i in range(3):
                self.cameracorrection[i] = self.target.position.value[0][i] - self.effector.getMechanicalState().position.value[0][i]
            self.rootnode.Simulation.Emio.CenterPart.Sensor.ForcePointActuator.applyForce=True

        # Update the target position of the effector
        # The target position is the position from the camera minus the correction
        self.effector.EffectorCoord.effectorGoal.value = [[self.target.position.value[0][0] - self.cameracorrection[0],
                                                            self.target.position.value[0][1] - self.cameracorrection[1],
                                                            self.target.position.value[0][2] - self.cameracorrection[2],
                                                            0., 0., 0., 1.]]
        
    def onAnimateEndEvent(self, _):
        # Update the force applied on the sensor
        dt = self.rootnode.dt.value
        self.sensor.Force.value = [self.sensor.ForcePointActuator.force.value[0] / dt,
                                   self.sensor.ForcePointActuator.force.value[1] / dt,
                                   self.sensor.ForcePointActuator.force.value[2] / dt]


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
    rootnode.VisualStyle.displayFlags = ["showVisual", "showInteractionForceFields"]

    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]

    # Add Emio to the scene
    emio = Emio(name="Emio",
                legsName=["sleg"],
                legsModel=["tetra"],
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
        emio.motors[i].addObject("RestShapeSpringsForceField", points=[0], stiffness=1e6)

    # Let's now add the components for the connection to the real robot 
    dotTracker = None
    if args.connection:

        # Motors controller and configuration
        simulation.addData(name="tilt", type="bool", value=False)
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
                                                       filter_alpha=0.5, # Factor used in the filter
                                                       rotation=emio.Camera.torealrotation,
                                                       translation=emio.Camera.torealtranslation))
        except RuntimeError as e:
            Sofa.msg_error(__file__, "Problem with the camera: " + str(e))

    # Add the position we want to observe (the effector)
    emio.effector.addObject("MechanicalObject", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]])
    emio.effector.addObject('PositionEffector', template="Rigid3", 
                            indices=[0],
                            useDirections=[1, 1, 1, 0, 0, 0],
                            limitShiftToTarget=True,
                            maxShiftToTarget=50,  # mm
                            effectorGoal=[0, 165, 0, 0, 0, 0, 1], 
                            name="EffectorCoord")
    emio.effector.addObject("RigidMapping", index=0)

    # Sensor (to retrieve the forces applied on the real robot effector)
    sensor = emio.centerpart.addChild("Sensor")
    sensor.addObject("MechanicalObject", position=[[0, 0, 0]])
    sensor.addObject("ForcePointActuator", 
                    name="ForcePointActuator",
                    indices=[0],
                    direction=[0, 0, 0],
                    applyForce=False, # We only want to apply the force when the assembly is done
                    showForce=True, 
                    visuScale=2
                    )
    sensor.addObject("RigidMapping", index=0)
    sensor.addData(name="Force", type="Vec3d", value=[0, 0, 0]) # We use this to store and send the sensed force through ROS2

    # GUI
    MyGui.SimulationState.addData("Sensor", "Force", sensor.Force) # We use this to send the force through ROS2
    MyGui.SimulationState.addData("TCP", "Frame", emio.effector.getMechanicalState().position) # We use this to send the position effector through ROS2
    MyGui.MyRobotWindow.addSetting("Configuration", simulation.tilt, 0, 1) # Add a setting ti the GUI to choose the configuration of the robot

    if dotTracker is not None:
        # Feed the effector target with the camera position
        rootnode.addObject(TargetController(rootnode=rootnode, 
                                            assemblycontroller=assemblycontroller, 
                                            sensor=sensor))

    return rootnode
