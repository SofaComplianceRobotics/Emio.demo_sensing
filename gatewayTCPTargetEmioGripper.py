#!/usr/bin/env python3

from math import atan2, sqrt, pi, cos, sin
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose



class GatewayTCPTargetEmioGripper(Node):

    def __init__(self, publishing_topic='/TCPTarget/Frame', subscribed_topic='/TCP/Frame'):
        super().__init__('gateway_emio_gripper_node')
        self.publish_topic = publishing_topic
        self.publisher = self.create_publisher(Float32MultiArray, 
                                               self.publish_topic, 
                                               10)

        self.subscribe_topic = subscribed_topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.subscribe_topic,
            self.listener_callback,
            10)
        
        self.get_logger().info(f'[GatewayTCPTargetEmioGripper] GatewayTCPTargetEmioGripper created with publisher on {self.publish_topic} and subscriber on {self.subscribe_topic}')
        self.last_position = [0, 0, 0]


    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'[GatewayTCPTargetEmioGripper] Received: {msg}')

        if len(msg.data) != 7:
            self.get_logger().error(f'[GatewayTCPTargetEmioGripper] Invalid message length: {len(msg.data)}')
            return

        # Convert compact Emio configuration to extended Emio configuration
        gripper_msg = Float32MultiArray()
        gripper_msg.data = [0.0] * 7
        
        # Position in mm
        eps = 2 # in mm
        shift_y = 350 # in mm
        gripper_msg.data[0] = msg.data[0] if abs(msg.data[0] - self.last_position[0]) > eps else self.last_position[0]
        gripper_msg.data[1] = msg.data[1] - shift_y if abs(msg.data[1] - self.last_position[1]) > eps else self.last_position[1] - shift_y
        gripper_msg.data[2] = msg.data[2] if abs(msg.data[2] - self.last_position[2]) > eps else self.last_position[2]

        if gripper_msg.data[1] < -250:
            gripper_msg.data[1] = -250
        elif gripper_msg.data[1] > -175:
            gripper_msg.data[1] = -175

        # Store the last position
        self.last_position[0] = gripper_msg.data[0] 
        self.last_position[1] = msg.data[1]
        self.last_position[2] = gripper_msg.data[2]

        # Orientation
        gripper_msg.data[3] = msg.data[3]
        gripper_msg.data[4] = msg.data[4]
        gripper_msg.data[5] = msg.data[5]
        gripper_msg.data[6] = msg.data[6]

        # Forward the message to the publisher
        self.publisher.publish(gripper_msg)
        self.get_logger().info(f'[GatewayTCPTargetEmioGripper] Forwarded: {gripper_msg}')


def main(args=None):
    rclpy.init(args=args)
    gateway = GatewayTCPTargetEmioGripper()
    rclpy.spin(gateway)
    gateway.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()