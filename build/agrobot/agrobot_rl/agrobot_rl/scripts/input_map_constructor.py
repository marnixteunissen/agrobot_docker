#! /usr/bin/env python3
import sys
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.duration import Duration

import numpy as np
from matplotlib import pyplot as plt
import cv2
from copy import copy
from pathlib import Path
import argparse
import time


class MapConstructor(Node):
    def __init__(self):
        super().__init__('map_constructor')
        # parse arguments:
        parser = argparse.ArgumentParser(
            description='This node uses the /map and /tf topics to create a custom'
                        'message containing a numpy array with a map that can be used'
                        'as the observation space of the OpenAI Gym.')
        parser.add_argument('-show', required=False, type=bool, default=True,
                            help='Option to visualize the map on screen')

        # self.args = parser.parse_args(args)
        self.show = True
        self.final_size = (200, 200)
        self.robot_trans = None
        self.rot_w = None
        self.transform_called = False
        self.map_save_loc = str(Path.home()) + '/agrobot_ws/src/agrobot/agrobot_rl/tmp/grid_map.png'

        # Initialize the tf listener:
        self.tf_buffer = Buffer(Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ogrid_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occupancy_grid_callback,
            10
        )
        # Initialize publishers:
        # TODO: create custom topic to publish map to
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.image_pub = self.create_publisher(
            Image,
            'grid_map',
            qos
        )
        self.bridge = CvBridge()

        if self.show:
            # Initialize map window:
            cv2.namedWindow('map', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('map', 400, 400)

    def get_map(self):
        '''
        This function returns the constructed map.
        The returned format is a Tensorflow tensor of size [1x3x200x200]
        '''
        map_ten = ...
        return map_ten

    def occupancy_grid_callback(self, msg):
        '''
        This function retrieves the occupancy grid map from the /map topic
        '''
        if msg is not None:
            # self.get_logger().info('I got a map!')
            self.grid_size = (msg.info.height, msg.info.width)
            self.grid = np.reshape(np.asarray(msg.data), self.grid_size)
            self.res = msg.info.resolution
            # the origin of the grid can change, therefore the origin is
            # included in the messages info, this will be used for the
            # translation of robot on the occupancy grid.
            self.grid_origin = {'position':     (msg.info.origin.position.x,
                                                 msg.info.origin.position.y,
                                                 msg.info.origin.position.z),
                                'orientation':  (msg.info.origin.orientation.x,
                                                 msg.info.origin.orientation.y,
                                                 msg.info.origin.orientation.z,
                                                 msg.info.origin.orientation.w)}
            # Scale grid values to 8bit
            # TODO: make sure the 'unknown' pixels don't give reward
            array = copy(self.grid)
            array[array == -1] = 50
            self.grid_png = (array * 2.55).astype(np.uint8)

            # Add colour channels:
            self.map_png = cv2.merge([self.grid_png, self.grid_png, self.grid_png])

            # Add the robots position to the map
            self.robot_transform_callback()
            if self.transform_called and self.robot_trans is not None:
                self.add_robot()

            # Make image properly sized:
            self.map_png = self.padding()
            self.map_png = cv2.flip(self.map_png, 1)
            cv2.imwrite(self.map_save_loc, self.map_png)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.map_png))
            if self.grid_size != (0, 0) and self.show:
                self.show_map()

    def robot_transform_callback(self):
        '''
        This function listens to the /tf topic, and uses the transformation
        between the odometry of the robot and the map.
        '''
        self.transform_called = True
        try:
            now = rclpy.time.Time()
            self.robot_trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                now
            ).transform.translation
        except TransformException as ex:
            self.get_logger().info(f'Transform from map to base_footprint not found: {ex}')
            self.transform_called = False

    def add_robot(self):
        '''
        This function adds a circle which represents the current location
        of the robot
        '''
        # Get the transforms from the map and the robot
        origin_trans = (self.grid_origin["position"][0], self.grid_origin["position"][1])
        # self.robot_transform_callback()

        # Calculate the translation from the map to the robots base
        robot_map_trans = (int(-(origin_trans[0] - self.robot_trans.x)/self.res),
                           int(-(origin_trans[1] - self.robot_trans.y)/self.res))

        # Adding circle at position of the robot:
        # TODO: add the orientation of the robot?
        try:
            self.map_png = cv2.circle(self.map_png, robot_map_trans, 1, (0, 0, 255))
        except:
            self.get_logger().warn('Invalid robot coordinates received, cannot plotting in map')

    def add_fruit(self):
        '''
        This function puts coloured dots on the map where fruit is located.
        Fruit is only added after the robots camera has 'seen' the fruit.
        '''
        ...

    def padding(self):
        '''
        This function makes sure the output image is of the size specified.
        '''
        (h, w) = self.grid_size
        (xx, yy) = self.final_size
        (ox, oy) = (-int(self.grid_origin['position'][0] / self.res),
                    -int(self.grid_origin['position'][1] / self.res))

        top = (yy // 2) - oy
        bottom = yy - top - h
        left = (xx // 2) - ox
        right = xx - left - w

        try:
            assert top + bottom + h == self.final_size[1] and left + right + w == self.final_size[0]
        except AssertionError:
            self.get_logger().info(f'assertion failed, got sizes: {left + right + w}, {top + bottom + h}')

        return cv2.copyMakeBorder(self.map_png, top, bottom, left, right, borderType=cv2.BORDER_REPLICATE)

    def show_map(self):
        cv2.imshow('map', self.map_png)
        cv2.waitKey(1)


def main(args=sys.argv):
    rclpy.init(args=args)

    map_constructor = MapConstructor()
    try:
        rclpy.spin(map_constructor)
        map_constructor.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Exitin map constructor node ...')
        map_constructor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
