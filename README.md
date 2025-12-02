# Emio.demo_sensing

This is a demo for the [Emio robot](https://compliance-robotics.com/compliance-lab/), showcasing its sensing capabilities. The directory contains the necessary files to run the demo with the application [Emio Labs](https://mailchi.mp/compliance-robotics.com/download-emio-labs).

In this demo we fix the motor to a desired configuration. We chose a configuration that let us use Emio as a joystick. 
On the real device, we track the position of the effector using a marker and the depth camera.
The problem we want to solve is to retrieve the force applied by the user on the effector of Emio. To achieve this, we solve the inverse problem in simulation.  

We can then use Emio as a joystick and ROS2 to control a turtle in the [TurtleSim](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) simulation environment, or another Emio robot!

![Emio in simulation](data/images/emio-sensing.png)

**Note**: The legs used in these demos are not distributed with Emio. You can find the files to print them yourself in the _data/meshes/legs/_ folder.  

**Instructions to print the legs**: Emio's legs (and gripper) are printed using TPU [filament](https://shop3d.ca/collections/flexible-filaments-tpu/products/bambu-lab-tpu-hf-1-75mm-1kg) with a Bambu Lab A1. Deactivate the AMS option on the Bambu Lab. For the infill, we recommend a 100% concentric pattern. You'll also get better results if you use a 0.6mm nozzle.
