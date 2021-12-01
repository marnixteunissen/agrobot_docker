#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_srvs.srv import Empty
from custom_interfaces.srv import EnvReset
from time import sleep
import signal
import os

from subprocess import PIPE, Popen, run


class EnvironmentResetService(Node):
    def __init__(self):
        super().__init__('environment_reset_service')
        self.cartographer_launch_cmd = 'ros2 launch agrobot_rl cartographer.launch.py'
        self.get_logger().info('Starting reset service...')
        self.req_nodes = ["cartographer",
                          "bt_navigator",
                          "environment_reset",
                          "gazebo",
                          "lifecycle_manager_navigation",
                          "occupancy_grid_node",
                          "planner_server",
                          "robot_state_publisher"
                          ]

        # Initialize services:
        self.srv = self.create_service(
            EnvReset,
            'reset_environment',
            self.env_reset_callback)

        self.kill_srv = self.create_service(
            EnvReset,
            'kill_navigation',
            self.kill_nav_callback
        )

        self.gazebo_reset = self.create_client(
            Empty,
            'reset_world')

        self.gazebo_pause = self.create_client(
            Empty,
            'pause_physics')

        self.gazebo_unpause = self.create_client(
            Empty,
            'unpause_physics')

        # initialize publishers:
        self.nav_goal = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        self.dyn_goal = self.create_publisher(
            PoseStamped,
            'goal_update',
            10
        )
        self.init_pose = PoseStamped()
        self.init_pose.header.frame_id = 'map'

        # Initialize the cartographer process as Pope object:
        self.cart_process = Popen('echo "Starting cartographer nodes!"',
                                  shell=True,
                                  stdout=PIPE)

        sleep(0.2)
        self.launch_cartographer()

    def env_reset_callback(self, request, response):
        try:
            # Kill the cartographer process
            self.kill_cartographer()
        except ProcessLookupError:
            self.get_logger().info('Subprocess was already killed')

        try:
            empty_req = Empty.Request()
            # Pause Gazebo:
            future_pause = self.gazebo_pause.call_async(empty_req)
            pause = self.check_future(future_pause, 'Environment was succesfully paused!')
            if not pause:
                self.get_logger().err(f'Simulation not successfully paused')

            # Reset Gazebo world:
            # future_reset = self.gazebo_reset.call_async(empty_req)
            # reset = self.check_future(future_reset, 'Environment was succesfully reset!')
            # if not reset:
            #     self.get_logger().err(f'Simulation not successfully reset')
            reset = True

            # Relaunch cartographer nodes:
            cart_launched = self.launch_cartographer()

            # Unpause the simulation
            future_unpause = self.gazebo_unpause.call_async(empty_req)
            restart = self.check_future(future_unpause, 'Environment succesfully restarted!')
            if not restart:
                self.get_logger().err(f'Simulation not successfully restarted')

            if pause and reset and restart and cart_launched and self.cart_process.poll() is None:
                response.success = True
            else:
                raise Exception("processes not properly reset")

        except:
            self.get_logger().err(f'Reset service not successfully executed.')
            self.kill_cartographer()
            response.success = False

        return response

    def launch_cartographer(self):
        if self.cart_process.poll() is None:
            self.get_logger().info(f'Cartographer still running '
                                   f'with pid: {self.cart_process.pid}')
            success = True
        else:
            self.cart_process = Popen("exec " + self.cartographer_launch_cmd,
                                      stdout=PIPE,
                                      preexec_fn=os.setsid,
                                      shell=True,)
            tries = 0
            while True:
                running_nodes = run('ros2 node list', shell=True, stdout=PIPE, stderr=PIPE).stdout.decode("utf-8")
                if all(x in running_nodes for x in self.req_nodes):
                    success = True
                    break
                elif tries >= 10:
                    not_run_node = [x for x in self.req_nodes if x not in running_nodes]
                    self.get_logger().info(f"Cartographer timed out after 10 tries, {not_run_node} not running")
                    self.get_logger().info("Killing cartographer...")
                    self.kill_cartographer()
                    success = False
                    break
                else:
                    self.get_logger().info("Cartographer not ready, waiting...")
                    tries += 1
                sleep(0.2)

        if self.cart_process.poll() is None:
            self.get_logger().info(f'Succesfully launched cartographer as subprocess '
                                   f'with pid: {self.cart_process.pid}')
            success = True

        return success

    def kill_cartographer(self):
        killed = False
        try:
            os.killpg(os.getpgid(self.cart_process.pid), signal.SIGKILL)
            while not killed:
                running_nodes = run('ros2 node list', shell=True, stdout=PIPE, stderr=PIPE)
                if self.cart_process.poll() is None:
                    self.get_logger().info(f'Subprocess not killed yet, waiting ...')
                elif "cartographer" in running_nodes.stdout.decode("utf-8"):
                    self.get_logger().info(f"Some cartographer nodes are still running...")
                else:
                    self.get_logger().info(f'Succesfully killed cartographer as subprocess '
                                           f'with pid: {self.cart_process.pid}')
                    killed = True
                    break
                sleep(2.0)
        except ProcessLookupError:
            self.get_logger().info(f'Cartographer process not found, '
                                   f'assuming it was already killed')

    def check_future(self, future, message):
        # success = False
        if future.result() is None:
            self.get_logger().info(message)
            success = True
        elif future.exception() is not None:
            self.get_logger().warn(f'Exception while calling service: {future.exception()}')
            success = False
        else:
            success = False
            self.get_logger().warn(f'Something went wrong...')

        return success

    def kill_nav_callback(self, request, response):
        try:
            # Kill the cartographer process
            self.kill_cartographer()
            killed = True
        except ProcessLookupError:
            self.get_logger().info('Subprocess was already killed')
            killed = False
        response.success = killed

        return response


def main(args=None):
    rclpy.init(args=args)

    reset_service = EnvironmentResetService()

    try:
        rclpy.spin(reset_service)
        reset_service.destroy_node()
        rclpy.shutdown()
    except:
        print("")
        print('catching exception')
        reset_service.kill_cartographer()
    finally:
        print('Destroying node...')
        reset_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
