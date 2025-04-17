#!/usr/bin/env python3

from math import atan2, sqrt, pi, cos, sin
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose



class GatewayTurtleNode(Node):

    def __init__(self, publishing_topic='/turtle1/cmd_vel', subscribed_topic='/Sensor/Force'):
        super().__init__('gateway_node')
        self.publish_topic = publishing_topic
        self.subscribe_topic = subscribed_topic
        self.publisher = self.create_publisher(Twist, self.publish_topic, 10)
        self.last_theta = 0.0

        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.subscribe_topic,
            self.listener_callback,
            10)
        
        self.subscriptionPose = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.get_logger().info(f'[GatewayNode] GatewayNode created with publisher on {self.publish_topic} and subscriber on {self.subscribe_topic}')
    
    def pose_callback(self, msg):
        self.last_theta = msg.theta

    def send_move_msg(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z ):
        msg = Twist()
        msg.linear = Vector3(x=linear_x, y=linear_y, z=linear_z)
        msg.angular = Vector3(x=angular_x, y=angular_y, z=angular_z)
        self.publisher.publish(msg)
        self.get_logger().info(f'[GatewayNode] send_move - Publishing: {msg.linear} {msg.angular} to {self.publish_topic}')

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'[GatewayNode] Received: {msg}')

        # Convert Float32MultiArray to Twist
        # Assuming msg.data is a list of 3 floats
        if len(msg.data) != 3:
            self.get_logger().error(f'[GatewayNode] Invalid message length: {len(msg.data)}')
            return
        twist_msg = Twist() 

        cartesian = msg.data
        # we rotate the coordinates of pi/4 to match the turtlebot's coordinates
        tetha = pi / 4
        cartesian[0] = cartesian[0] * cos(tetha) - cartesian[2] * sin(tetha)
        cartesian[2] = cartesian[0] * sin(tetha) + cartesian[2] * cos(tetha)

        # we scale the force 
        scaling_factor = 1e-3 

        # convert the cartesian coordinates to polar coordinates
        polar = [sqrt(cartesian[0]**2 + cartesian[2]**2)*scaling_factor, atan2(cartesian[2], -cartesian[0])]

        self.get_logger().info(f'[GatewayNode] Polar coordinates: {polar}')

        # Filter the linear velocity
        if abs(polar[0]) < 0.5:
            polar[0] = 0.0

        # Get the last polar coabordinates of turtlebot and compute angle difference with the new polar coordinates
        polar[1] = polar[1] - self.last_theta

        # Normalize polar to be between -pi and pi
        if polar[1] > pi:
            polar[1] -= pi * 2.0
        elif polar[1] < -pi:
            polar[1]  += pi*2.0

        self.get_logger().info(f'[GatewayNode] Last Theta {self.last_theta}')

        # Set the linear and angular velocities
        twist_msg.linear.x = polar[0]
        twist_msg.angular.z = polar[1]

        # Forward the message to the publisher
        self.send_move_msg(
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z
        )
        self.get_logger().info(f'[GatewayNode] Forwarded: {twist_msg.linear} {twist_msg.angular}')


def main(args=None):
    rclpy.init(args=args)
    gateway = GatewayTurtleNode()
    rclpy.spin(gateway)
    gateway.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()