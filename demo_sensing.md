# Demo Sensing

::: highlight
##### Overview 

In this demo, we will explore the sensing capabilities of Emio. 
:::

:::: collapse Sensing

## Sensing

Emio can be use as a sensor. In this demo we fix the motor to a desired configuration. We chose a configuration that let us use Emio as a joystick. 
On the real device, we track the position of the effector using a marker and the depth camera.
The problem we want to solve is to retrieve the force applied by the user on the effector of Emio. To achieve this, we solve the inverse problem in simulation. 

We can then use Emio as a joystick and ROS2 to control a turtle in the [TurtleSim](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) simulation environment.

### Turtle sim demo

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
Run the simulation by clicking the play button (top middle of the 3D view). You can now play with the demo. 

#runsofa-button("assets/labs/demo_sensing/demo_sensing.py")

::::