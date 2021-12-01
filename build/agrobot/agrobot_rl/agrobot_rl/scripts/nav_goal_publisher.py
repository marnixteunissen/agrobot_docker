#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from custom_interfaces.srv import PubGoal

import numpy as np
from pprint import pprint


class NavigationGoal(Node):
    def __init__(self):
        super().__init__('navigation_goal_pub')
        # Create publisher for goal_pose topic
        self.nav_pub = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        # create service:
        self.nav_srv = self.create_service(
            PubGoal,
            'set_goal',
            self.nav_srv_callback
        )
        self.empty_nav_goal = PoseStamped()

    def nav_srv_callback(self, request, response):
        quat = self.euler_to_quaternion(request.w)

        self.get_logger().info(f'Received navigation goal: '
                               f'[x: {request.x}, y: {request.y}, w: {request.w}]')

        nav_goal = self.empty_nav_goal
        nav_goal.header.frame_id = 'map'
        nav_goal.pose.position.x = request.x
        nav_goal.pose.position.y = request.y
        nav_goal.pose.orientation.z = quat['z']
        nav_goal.pose.orientation.w = quat['w']

        try:
            self.nav_pub.publish(nav_goal)
            response.published = True
        except:
            response.published = False

        return response

    def euler_to_quaternion(self, euler_w):
        """
          Converts an Euler angle to a quaternion, simplified for a 2D application
          like setting navigation pose of a mobile robot.

          Input
            :param euler_w: he yaw (rotation around z-axis) angle in radians.

          Output
            :return dict with {x: qx, y: qy, z: qz, w: qw}:
                The orientation in quaternion [x,y,z,w] format
          """
        qz = np.sin(euler_w / 2)
        qw = np.cos(euler_w / 2)
        
        return {'x': 0, 'y': 0, 'z': qz, 'w': qw}


def main(args=None):
    rclpy.init(args=args)

    nav_goal_service = NavigationGoal()

    try:
        rclpy.spin(nav_goal_service)
    except KeyboardInterrupt:
        print('Exiting navigation goal node ...')
        nav_goal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
