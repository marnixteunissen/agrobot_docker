#! /usr/bin/env python3

import gym
import os
import cv2
import signal
from gym import error, spaces, utils
from gym.utils import seeding
from subprocess import Popen, run, PIPE
from time import sleep
from math import pi
import numpy as np


class SimStartTest():
    def __init__(self):
        self.start = True
        self.map_path = '/home/mtn/agrobot_ws/src/agrobot/agrobot_rl/tmp/grid_map.png'
        self.req_nodes = ["cartographer",
                          "bt_navigator",
                          "environment_reset",
                          "gazebo",
                          "grid_map",
                          "lifecycle_manager_navigation",
                          "occupancy_grid_node",
                          "planner_server",
                          "robot_state_publisher"
                          ]
        self.running = False
        self.start_ros_simulation()
        print("start sim function done")

    def start_ros_simulation(self):
        print("Starting navigation subprocess:")
        self.nav_proc = Popen("exec ros2 run agrobot_rl world_reset_node.py",
                              shell=True,
                              preexec_fn=os.setsid,
                              stdout=PIPE)
        print("Starting simulation subprocess:")
        self.sim_proc = Popen("exec ros2 launch agrobot_rl rl_env_gazebo.launch.py",
                              shell=True,
                              preexec_fn=os.setsid,
                              stdout=PIPE)
        while not self.running:
            running_nodes = run('ros2 node list', shell=True, stdout=PIPE).stdout.decode("utf-8")
            if all(x in running_nodes for x in self.req_nodes):
                self.running = True
                break
            else:
                print("Not all nodes are running, waiting...")
            sleep(1.5)
        if self.sim_proc.poll() is None and self.nav_proc.poll() is None:
            print('Simulation and navigation stack are running!')

    def kill_ros_simulation(self):
        killed = False
        print('Killing simulation and navigation processes...')
        self.kill_srv()
        try:
            os.killpg(os.getpgid(self.sim_proc.pid), signal.SIGTERM)
            os.killpg(os.getpgid(self.nav_proc.pid), signal.SIGTERM)
            while not killed:
                running_nodes = run('ros2 node list', shell=True, stdout=PIPE)
                if self.sim_proc.poll() is None and self.nav_proc.poll() is None:
                    print('Processes not killed, waiting...')
                elif running_nodes.stdout.decode("utf-8"):
                    print("Some nodes are still running...")
                else:
                    print('Succesfully killed simulation and navigation processes!')
                    self.running = False
                    break
                sleep(2.0)
        except ProcessLookupError:
            print("Simulation process not found, assuming it was killed.")
        except Exception as ex:
            print(f"An exception occured while killing the subprocesses: {ex}")

    def kill_srv(self):
        result = run('ros2 service call /kill_navigation custom_interfaces/srv/EnvReset',
                     shell=True,
                     stdout=PIPE)
        if "True" in result.stdout.decode("utf-8"):
            print('navigation killed')
        else:
            print('navigation not killed')

    def reset(self):
        # reset the gazebo simulation through the ROS service call
        success = False
        while not success:
            result = run("ros2 service call /reset_environment custom_interfaces/srv/EnvReset",
                         shell=True,
                         stdout=PIPE)
            if "True" in result.stdout.decode("utf-8"):
                success = True
                # Wait for the simulation to reset
                sleep(5.0)
                observation = self.get_obs()
            else:
                success = False
                print('Environment was not succesfully reset, trying again...')
                sleep(1.0)

        return observation

    def get_obs(self):
        # Function to get an observation from the ROS framework
        observation = None
        while observation is None:
            try:
                observation = cv2.imread(self.map_path)
            except:
                observation = None
                sleep(0.05)

        return observation


def main():
    print("starting sim proces...")
    simcontrol = SimStartTest()
    try:
        while True:
            print("still running..............")
            sleep(20.0)
            print('resetting...')
            image = simcontrol.reset()
            print(image)
            cv2.imshow('input map', image)
            cv2.waitKey(1)
    except Exception as ex:
        print(f'caught exception: {ex}')
    finally:
        print("")
        print("starting kill process...")
        simcontrol.kill_ros_simulation()


if __name__ == '__main__':
    try:
        main()
    finally:
        cv2.destroyAllWindows()
