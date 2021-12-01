#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

import numpy as np
from pprint import pprint


class TransformSubscriberBase(Node):

    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg is not None:
            if msg.transforms[0].header.frame_id == 'map' and msg.transforms[0].child_frame_id == 'odom':
                self.get_logger().info('I heard a transform!')
                print(msg.transforms[0].transform.translation)


def main(args=None):
    rclpy.init(args=args)

    map_subscriber = TransformSubscriberBase()

    rclpy.spin(map_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
