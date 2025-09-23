import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../../")

import os.path
import Sofa


def createScene(rootnode):
    from utils.header import addHeader, addSolvers
    from parts.gripper import Gripper
    from parts.controllers.assemblycontroller import AssemblyController
    import Sofa.ImGui as MyGui
    from parts.emio import Emio
    from parts.controllers.trackercontroller import DotTracker

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    addSolvers(simulation)

    rootnode.dt = 0.03
    rootnode.gravity = [0., -9810., 0.]
    rootnode.VisualStyle.displayFlags.value = ["hideBehavior", "hideWireframe"]

    # Add Emio to the scene
    emio = Emio(name="Emio",
                legsName=["longleg"],
                legsModel=["beam"],
                legsPositionOnMotor=["counterclockwisedown", "clockwisedown", "counterclockwisedown", "clockwisedown"],
                centerPartName="whitepart",
                centerPartType="deformable",
                centerPartModel="beam",
                centerPartClass=Gripper,
                extended=True)
    if not emio.isValid():
        return

    simulation.addChild(emio)
    emio.attachCenterPartToLegs()
    emio.addObject(AssemblyController(emio))

    # Add effector
    emio.effector.addObject("MechanicalObject", template="Rigid3", position=[0, 0, 0, 0, 0, 0, 1] * 4)
    emio.effector.addObject("RigidMapping", rigidIndexPerPoint=[0, 1, 2, 3])

    # Target
    effectorTarget = modelling.addChild('Target')
    effectorTarget.addObject('EulerImplicitSolver', firstOrder=True)
    effectorTarget.addObject('CGLinearSolver', iterations=50, tolerance=1e-10, threshold=1e-10)
    effectorTarget.addObject('MechanicalObject', template='Rigid3',
                             position=[0, -170, 0, 0, 0, 0, 1],
                             showObject=True, showObjectScale=20)

    # Add inverse components and GUI
    emio.addInverseComponentAndGUI(effectorTarget.getMechanicalState().position.linkpath, barycentric=True)
    emio.effector.EffectorCoord.maxSpeed.value = 20  # mm/s
    MyGui.MoveWindow.setTCPLimits(-100, 100,
                                    emio.motors[0].JointActuator.minAngle.value,
                                    emio.motors[0].JointActuator.maxAngle.value)

    TCP = modelling.addChild("TCP")
    TCP.addObject("MechanicalObject", template="Rigid3", position=emio.effector.EffectorCoord.barycenter.linkpath)
    MyGui.setIPController(rootnode.Modelling.Target, TCP, rootnode.ConstraintSolver)

    # GUI slider for the gripper opening
    MyGui.MoveWindow.addAccessory("Gripper's opening (mm)", emio.centerpart.Effector.Distance.DistanceMapping.restLengths, 8, 70)
    MyGui.ProgramWindow.addGripper(emio.centerpart.Effector.Distance.DistanceMapping.restLengths, 8, 70)
    MyGui.IOWindow.addSubscribableData("/Gripper", emio.centerpart.Effector.Distance.DistanceMapping.restLengths)

    # Components for the connection to the real robot 
    emio.addConnectionComponents()
        
    return rootnode
