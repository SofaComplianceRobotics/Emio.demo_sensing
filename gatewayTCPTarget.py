#!/usr/bin/env python3

from math import atan2, sqrt, pi, cos, sin
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose



class GatewayEmioJoystick(Node):

    def __init__(self, 
                 publishing_topic='/TCPTarget/Frame', 
                 subscribed_topic_force='/Sensor/Force', 
                 subscribed_topic_position='/TCP/Frame'):
        
        super().__init__('gateway_emio_joystick_node')
        self.publishing_topic = publishing_topic
        self.publisher = self.create_publisher(Float32MultiArray, 
                                               self.publishing_topic, 
                                               10)

        self.subscribed_topic_force = subscribed_topic_force
        self.subscription_force = self.create_subscription(
            Float32MultiArray,
            self.subscribed_topic_force,
            self.listener_callback_force,
            10)
        
        self.subscribed_topic_position = subscribed_topic_position
        self.subscription_position = self.create_subscription(
            Float32MultiArray,
            self.subscribed_topic_position,
            self.listener_callback_position,
            10)
        
        self.get_logger().info(f'[GatewayEmioJoystick] Created with publisher on {self.publishing_topic} and subscribers on {self.subscribed_topic_force} and {self.subscribed_topic_position}')
        self.last_position = [0., 0., 0., 0., 0., 0., 1.]


    def listener_callback_force(self, msg):
        # Log the received message
        # self.get_logger().info(f'[GatewayEmioJoystick] Received force: {msg}')

        if len(msg.data) != 3:
            self.get_logger().error(f'[GatewayEmioJoystick] Invalid message length: {len(msg.data)}')
            return

        print("Listening Force:", msg.data)

        # Copy last position
        force_msg = msg.data
        position = [0., 0., 0., 0., 0., 0., 1.]
        for i in range(7):
            position[i] = self.last_position[i]
        
        # Don't apply small forces
        for i in range(3):
            if -1000.0 < force_msg[i] < 1000.0:
                force_msg[i] = 0.0
            else:
                force_msg[i] = force_msg[i] * 5e-3  # Scale down the force
        if force_msg[0] == 0.0 and force_msg[1] == 0.0 and force_msg[2] == 0.0:
            return

        # Position in mm
        position[0] += force_msg[0]
        position[1] += force_msg[1] 
        position[2] += force_msg[2] 

        # Clamp the position values to the range [-250, -175] in y direction
        position[0] = max(-40, min(position[0], 40))
        print(position[1])
        print(force_msg[1])
        position[1] = max(-250, min(position[1], -175))
        position[2] = max(-40, min(position[2], 40))

        # Forward the message to the publisher
        position_msg = Float32MultiArray()
        position_msg.data = position
        print("Publishing position:", position_msg.data)
        self.publisher.publish(position_msg)



    def listener_callback_position(self, msg):
        # Log the received message
        # self.get_logger().info(f'[GatewayEmioJoystick] Received: {msg}')

        if len(msg.data) != 7:
            self.get_logger().error(f'[GatewayEmioJoystick] Invalid message length: {len(msg.data)}')
            return

        print("Listening position:", msg.data)
        for i in range(7):
            self.last_position[i] = msg.data[i]


def main(args=None):
    rclpy.init(args=args)
    gateway = GatewayEmioJoystick()
    rclpy.spin(gateway)
    gateway.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()