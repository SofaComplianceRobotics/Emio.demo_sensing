# Demo Sensing

::: highlight
##### Overview 

In this demo, we will explore the sensing capabilities of Emio. 

:::

:::: collapse Force Estimation Demo

### Force Estimation Demo

|![Simulation of the force estimation](assets/labs/demo_sensing/data/images/emio-sensing.png){width=80%}|
|:--------------------------------------------------------------------:|
|**Simulation of the force estimation.**                               |

Emio can be use as a sensor. In this demo we fix the motor to a desired configuration. We chose a configuration that let us use Emio as a joystick. 
On the real device, we track the position of the effector using a marker and the depth camera.
The problem we want to solve is to retrieve the force applied by the user on the effector of Emio. To achieve this, we solve the inverse problem in simulation. 

Run the simulation by clicking the button below, then press the play button to start the simulation (top middle of the 3D view). Set up the robot Emio like in the simulation, [connect the robot to your computer](https://docs-support.compliance-robotics.com/docs/next/Users/Emio/getting-started-with-emio/#connecting-emio-to-your-computer) and then [connect the simulation](https://docs-support.compliance-robotics.com/docs/next/Users/SOFARobotics/GUI-user-manual/#controlling-the-real-robot). You can now play with the demo. Move the effector of the real robot and observe the estimation of the force in the simulation. 

#runsofa-button("assets/labs/demo_sensing/demo_sensing.py")

::::

:::: collapse Turtle Sim Demo

### Turtle Sim Demo

We can use Emio as a joystick and ROS2 to control a turtle in the [TurtleSim](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) simulation environment.

In a terminal:
```command
source /opt/ros/jazzy/setup.sh
ros2 run turtlesim turtlesim_node
```

In another terminal, from the demo_sensing directory:
```command
source /opt/ros/jazzy/setup.sh
pyenv local 3.12
python gatewayTurtleNode.py
```

On the GUI, select the window __Windows>Input/Output__. From this window __Input/Output__, in the output section, select the topic __Sensor/Force__ and click toggle the __Publish__ button.

::::

:::: collapse Teleoperation Demo

### Teleoperation Demo

We can use Emio as a joystick to control a second Emio robot, still using ROS2.

In a terminal, from the demo_sensing directory:
```command
source /opt/ros/jazzy/setup.sh
pyenv local 3.12
python gatewayTCPTargetEmioGripper.py
```

This gateway script transforms the force estimated in the first simulation into a TCP target for the second robot.
This script subscribes to the topic __Sensor/Force__ and publishes on the topic __TCPTarget/Frame__.

In the first simulation (Force Estimation Demo), publish the force on the topic __Sensor/Force__ by first clicking the corresponding checkbox, and then clicking the button __Publish__ in the window __Windows>Input/Output__.

Run a second simulation by clicking the button below, then press the play button to start the simulation. Set up the second robot Emio like in the simulation, [connect the robot to your computer](https://docs-support.compliance-robotics.com/docs/next/Users/Emio/getting-started-with-emio/#connecting-emio-to-your-computer) and then [connect the simulation](https://docs-support.compliance-robotics.com/docs/next/Users/SOFARobotics/GUI-user-manual/#controlling-the-real-robot). 
In the window __Windows>Input/Output__, in the input section, select the topic __TCPTarget/Frame__ and click toggle the __Subscribe__ button.
Select "IOWindow" from the menu in the 3D view, this will allow the window __Windows>Input/Output__ to control the TCP target in the simulation. 
You can now play with the demo. Move the effector of the first real robot and observe the motion of the second robot.

#runsofa-button("assets/labs/demo_sensing/demo_teleoperation.py")

::::