#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseSubscriber(Node):
    
    def __init__(self):
        super().__init__("pose_sub")
        self._pose_subscriber=self.create_subscription(
            Pose, "/turtle1/pose",self.pose_subscriber,10)


    def pose_subscriber(self,pose:Pose):
        self.get_logger().info("[X="+str(pose.x)+"Y="+str(pose.y)+"]")


def main(args=None):
    rclpy.init(args=args)
    node=PoseSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()