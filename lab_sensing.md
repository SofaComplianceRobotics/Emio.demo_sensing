# Lab Sensing

::: highlight
##### Overview 

In this lab, we will explore the sensing capabilities of Emio. 
:::

:::: collapse Sensing

## Sensing

Emio can be use as a sensor. In this lab we fix the motor to a desired configuration. We chose a configuration that let us use Emio as a joystick. 
We track the position of the effector using a marker and the depth camera.
The problem we want to solve is to retreive the force applied by the user on the effector of Emio.  

### Turtle sim demo

In a terminal:
```command
source /opt/ros/jazzy/setup.sh
ros2 run turtlesim turtlesim_node
```

In another terminal:
```command
source /opt/ros/jazzy/setup.sh
pyenv local python3.12
python gatewayTurtleNode.py
```

#runsofa-button("assets/labs/lab_sensing/lab_sensing.py")

::::