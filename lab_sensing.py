import os
import sys
import Sofa.ImGui as MyGui
import Sofa.Core

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../EmioLabs/assets/")

class Controller(Sofa.Core.Controller):

    def __init__(self, rootnode, assemblycontroller):
        Sofa.Core.Controller.__init__(self)
        self.rootnode = rootnode
        self.target = rootnode.DepthCamera.getMechanicalState()
        self.effector = rootnode.Simulation.Emio.CenterPart.Effector
        self.assemblycontroller = assemblycontroller

        self.correctiondone = False
        self.cameracorrection = [0, 0, 0]

    def onAnimateBeginEvent(self, _):
        self.effector.EffectorCoord.effectorGoal.value = [[self.target.position.value[0][0] - self.cameracorrection[0],
                                                           self.target.position.value[0][1] - self.cameracorrection[1],
                                                           self.target.position.value[0][2] - self.cameracorrection[2]]]
        if not self.correctiondone and self.assemblycontroller.done:
            self.correctiondone = True
            for i in range(3):
                self.cameracorrection[i] = self.target.position.value[0][i] - self.effector.getMechanicalState().position.value[0][i]

def getParserArgs():
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
    from utils import getListFromArgs
    from parts.controllers.assemblycontroller import AssemblyController
    from parts.emio import Emio

    args = getParserArgs()

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.VisualStyle.displayFlags = ["showInteractionForceFields","showVisual"]

    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    addSolvers(simulation)

    # Add Emio to the scene
    emio = Emio(name="Emio",
                legsName=["blueleg"],
                legsModel=["beam"],
                legsPositionOnMotor=["clockwiseup", "counterclockwiseup", "clockwiseup", "counterclockwiseup"],
                centerPartName="yellowpart",
                centerPartType="rigid",
                extended=False)
    if not emio.isValid():
        return

    simulation.addChild(emio)
    emio.attachCenterPartToLegs()
    assemblycontroller = AssemblyController(emio)
    emio.addObject(assemblycontroller)

    # Components for the connection to the real robot 
    dotTracker = None
    if args.connection:
        emio.addConnectionComponents()

        try:
            from parts.controllers.trackercontroller import DotTracker
            dotTracker = rootnode.addObject(DotTracker(name="DotTracker",
                                                       root=rootnode,
                                                       nb_tracker=1,
                                                       show_video_feed=False,
                                                       track_colors=True,
                                                       comp_point_cloud=False,
                                                       scale=1,
                                                       rotation=emio.Camera.torealrotation,
                                                       translation=emio.Camera.torealtranslation))
        except RuntimeError:
            Sofa.msg_error(__file__, "Camera not detected")

    if dotTracker is not None:
        # Effector
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
        sensor.addObject("ForcePointActuator", indices=[0],
                         applyForce=False, # just estimating the force not applying it
                         showForce=True,
                         visuScale=1
                        )
        sensor.addObject("RigidMapping", index=0)

        # Feed the effector target with the camera position
        rootnode.addObject(Controller(rootnode, assemblycontroller))

        # Simulation State Tab
        MyGui.SimulationState.addData("Sensor", "Force", emio.effector.EffectorCoord.effectorGoal)

    return rootnode
