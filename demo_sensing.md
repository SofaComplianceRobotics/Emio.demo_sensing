# Demo Sensing

::: highlight
##### Overview 

In this demo, we will explore the sensing capabilities of Emio. 

:::

:::: collapse Force Estimation Demo

### Force Estimation Demo

|![Simulation of the force estimation](assets/labs/demo_sensing/data/images/emio-sensing.png){width=100%}|
|:--------------------------------------------------------------------:|
|**Simulation of the force estimation.**                               |

Emio can be use as a sensor. In this demo we fix the motor to a desired configuration. We chose a configuration that let us use Emio as a joystick. 
On the real device, we track the position of the effector using a marker and the depth camera.
The problem we want to solve is to retrieve the force applied by the user on the effector of Emio. To achieve this, we solve the inverse problem in simulation. 

1. Run the simulation by clicking the button below, then press the play button to start the simulation (top middle of the 3D view). Set up the robot Emio like in the simulation, [connect the robot to your computer](https://docs-support.compliance-robotics.com/docs/next/Users/Emio/getting-started-with-emio/#connecting-emio-to-your-computer) and then [connect the simulation](https://docs-support.compliance-robotics.com/docs/next/Users/SOFARobotics/GUI-user-manual/#controlling-the-real-robot). You can now play with the demo. Move the effector of the real robot and observe the estimation of the force in the simulation. 

    #runsofa-button("assets/labs/demo_sensing/demo_sensing.py")

::::

:::: collapse Turtle Sim Demo

### Turtle Sim Demo

|![Publishing the force using ROS2, and piloting the turtle sim.](assets/labs/demo_sensing/data/images/emio-sensing-turtlesim.png){width=100%}|
|:--------------------------------------------------------------------:|
|**Publishing the force using ROS2, and piloting the turtle sim.**                               |

We can use Emio as a joystick and ROS2 to control a turtle in the [TurtleSim](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) simulation environment.

1. In a first terminal, run the turtlesim node:
    ```command
    source /opt/ros/jazzy/setup.sh
    ros2 run turtlesim turtlesim_node
    ```

2. In another terminal, from the demo_sensing directory, run the gateway script:
    ```command
    source /opt/ros/jazzy/setup.sh
    pyenv local 3.12
    python gatewayTurtleNode.py
    ```

3. Launch the simulation by clicking the Run SOFA button below. On the GUI (see image above):
    1. From the __Input/Output__ window, in the __Output__ section, select the topic __Sensor/Force__
    2. and then click the __Publish__ button 
    #runsofa-button("assets/labs/demo_sensing/demo_sensing.py")


::::

:::: collapse Tele-operation Demo

### Tele-operation Demo

We can use Emio as a joystick to control a second Emio robot, still using ROS2.

|![Publishing the force using ROS2.](assets/labs/demo_sensing/data/images/emio-sensing-publishforce.png){width=100%}|
|:--------------------------------------------------------------------:|
|**Image 1. First simulation, publishing the force using ROS2.**                               |

|![Subscription.](assets/labs/demo_sensing/data/images/emio-sensing-teleoperation.png){width=100%}|
|:--------------------------------------------------------------------:|
|**Image 2. Second simulation subscription to the TCP target publised by the GatewayTCPTarget script.**|

1. In a terminal, from the demo_sensing directory, run the gateway script:
    ```command
    source /opt/ros/jazzy/setup.sh
    pyenv local 3.12
    python gatewayTCPTarget.py
    ```

    _This gateway script transforms the force estimated in the first simulation into a TCP target for the second robot.
    This script subscribes to the topics __Sensor/Force__ and __TCP/Frame__ and publishes on the topic __TCPTarget/Frame__._

2. In the first simulation (Run SOFA button below), publish the force (see Image 1.):
    1. From the __Input/Output__ window, in the __Output__ section, click the checkbox __Sensor/Force__
    2. And then click the __Publish__ button

    #runsofa-button("assets/labs/demo_sensing/demo_sensing.py")  

3. Run a second simulation by clicking the button below, then press the play button to start the simulation. Set up the second robot Emio like in the simulation, [connect the robot to your computer](https://docs-support.compliance-robotics.com/docs/next/Users/Emio/getting-started-with-emio/#connecting-emio-to-your-computer) and then [connect the simulation](https://docs-support.compliance-robotics.com/docs/next/Users/SOFARobotics/GUI-user-manual/#controlling-the-real-robot).  

    Now set up the GUI (see Image 2.). From the __Input/Output__ window:

    1. In the __Output__ section, click the checkbox __TCP/Frame__
    2. And then click the __Publish__ button 
    3. In the __Input__ section, click the checkbox __TCPTarget/Frame__
    4. And then click the __Subscribe__ button   
    5. Now in the 3D view, select __Input/Output__ from the dropdown menu.  
    _This will allow the __Input/Output__ window to control the TCP target in the simulation._ 
    
    #runsofa-button("assets/labs/demo_sensing/demo_teleoperation.py")  

You can now play with the demo. Move the effector of the _Joystick_ robot and observe the motion of the _Manipulator_ robot.

::::