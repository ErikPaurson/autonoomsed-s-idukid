#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("controller")
        self.get_logger().info("our controller is started")

        self._pose_publisher=self.create_publisher(
            Twist,"/turtle1/cmd_vel",10)
        
        self._pose_listener=self.create_subscription(
            Pose,"/turtle1/pose",self.robot_controller,10)
        
    def robot_controller(self,pose:Pose):
        cmd=Twist()

        if 1.0<pose.x<9.0 and 1.0<pose.y<9.0:

            cmd.linear.x=5.0
            cmd.angular.z=0.0
        else:
            cmd.linear.x=1.0
            cmd.angular.z=1.0

        self._pose_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node=TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
