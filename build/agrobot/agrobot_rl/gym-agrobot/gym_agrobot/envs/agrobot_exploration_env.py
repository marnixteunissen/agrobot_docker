import gym
import os
import cv2
import signal
from gym import error, spaces, utils
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import DQN, PPO, A2C
from stable_baselines3.common.evaluation import evaluate_policy
import tensorflow as tf
from gym.utils import seeding
from subprocess import Popen, run, PIPE
from time import sleep
from math import pi
from datetime import datetime
from itertools import compress
from copy import copy
from pathlib import Path
import numpy as np


class AgrobotExploreEnv(gym.Env):
    def __init__(self):
        self.map_path = str(Path.home()) + '/agrobot_ws/src/agrobot/agrobot_rl/tmp/grid_map.png'
        self.req_nodes = ["cartographer",
                          "bt_navigator",
                          "environment_reset",
                          "gazebo",
                          "lifecycle_manager_navigation",
                          "occupancy_grid_node",
                          "planner_server",
                          "robot_state_publisher"
                          ]
        self.running = False
        self.headless = True
        self.robot_spawned = False
        self.use_rviz = True
        self.rviz_config = 'rviz_during_training.rviz'

        # Launch Simulation, and reset node in separate threads:
        try:
            while not self.robot_spawned:
                self.start_ros_simulation()
                if not self.robot_spawned:
                    self.kill_ros_simulation()
                else:
                    break
        except KeyboardInterrupt:
            self.kill_ros_simulation()
            raise SystemExit

        print('Everything is working!')
        # Adding timeout to ensure simulation is running:
        #sleep(10.0)

        # Defining the action space:
        bounds_low = (-5.0, -5.0, 0.0)     # [min_x, min_y, min_w]
        bounds_high = (5.0, 5.0, float(2*pi))    # [max_x, max_y, min_w]
        self.action_space = spaces.Box(np.array(bounds_low), np.array(bounds_high), shape=(3,))

        # Defining the observation space:
        self.obs_shape = (200, 200, 3)
        self.observation_space = spaces.Box(low=0, high=255, shape=self.obs_shape, dtype=np.uint8)

        # Initializing parameters
        self.done = False
        self.info = {}
        self.reward = 0
        self.old_state = np.full(self.obs_shape, 127)
        self.nr_fruit = 0
        self.tot_fruit = 0
        self.new_fruit = None
        self.no_change = 0
        self.max_no_change = 6
        self.cont_area = None
        self.old_cont_area = None
        self.last_map_value = None
        self.failed_ep = 0
        # These parameter can be used to tweak the reward function:
        self.min_change = 50.0
        self.max_change = 500.0
        self.map_complete_reward = 20
        self.fruit_rew = 1
        self.step_dicount = 0.5
        self.cont_factor = 200.0

    def step(self, action):
        cmd = 'exec ros2 topic pub /goal_pose geometry_msgs/PoseStamped "'
        cmd += '{header: {stamp: {sec: 0}, frame_id: "map"}, pose: {position: {'
        cmd += 'x: ' + str(action[0]) + ', '
        cmd += 'y: ' + str(action[1]) + ', z: 0.0}, orientation: {'
        cmd += 'w: ' + str(action[2]) + '}}}" -1'
        # Publish the navigation goal to ros through the /set_goal service:
        print(f"Navigating to: (x={action[0]} y={action[1]} w={action[2]})")
        run(cmd, shell=True, stdout=PIPE)
        self.last_map_value = np.sum(np.abs(self.state - 127) / 127)
        self.nav_pixel = self.state[int((action[0]+5)*20), int((action[1]+5)*20)]

        # this delay is to make sure that ros has received the new goal and to get the robot moving
        sleep(0.5)

        self.state = copy(self.get_obs())

        # Task is done when:
        # The map forms a closed contour
        self.map_complete, self.cont_area = self.check_contour()
        # When all fruits have been found:
        all_fruit_found = self.check_fruit()
        # Or when the new map is the exact same as the previous few observations:
        map_changed = np.sum((self.old_state/255) - (self.state/255)) >= self.min_change
        if not map_changed:
            self.no_change += 1
        else:
            self.no_change = 0
        self.done = (self.map_complete and all_fruit_found) or (self.no_change >= self.max_no_change)
        if self.done and (self.map_complete and all_fruit_found):
            print("Done, the map was complete and all fruit was found!")
        elif self.done and (self.no_change >= self.max_no_change):
            print(f'Done, the map did not change enough over the last {self.max_no_change} steps')
            self.failed_ep += 1

        # Get the reward based on the current observation:
        self.reward = self.get_reward()
        self.info = {}

        # Set the current observation as previous observation for next reward calculation:
        self.old_state = copy(self.state)
        self.old_cont_area = self.cont_area
        self.last_map_value = np.sum(np.abs(self.state - 127) / 127)

        return self.state, self.reward, self.done, self.info

    def reset(self):
        # reset the gazebo simulation through the ROS service call
        print("Requesting reset of environment...")
        success = False
        while not success:
            result = run("ros2 service call /reset_environment custom_interfaces/srv/EnvReset",
                         shell=True,
                         stdout=PIPE)
            if "True" in result.stdout.decode("utf-8"):
                success = True
                self.no_change = 0
                # Wait for the simulation to reset
                sleep(2.8)
                self.state = self.get_obs()
                self.old_state = self.state
                _, self.old_cont_area = self.check_contour()
                self.last_map_value = np.sum(np.abs(self.state - 127) / 127)
            else:
                success = False
                print('Environment was not succesfully reset, trying again...')
                sleep(1.0)

        return self.state

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

    def get_reward(self):
        # Function to get reward based on the current observation and action
        # Reward for finding new fruit (is 0 until implemented):
        fruit_reward = self.fruit_rew * self.new_fruit

        # Reward for increasing knowledge of the map:
        new = np.sum(np.abs(self.state - 127)/127)
        map_reward = np.clip((new - self.last_map_value) / self.max_change, 0, None)

        if self.map_complete:
            map_reward += self.map_complete_reward

        # The contour reward rewards increase of the area of the contours,
        # which are the edges of the map or the area of the map once it's completed
        contour_reward = np.clip((self.cont_area - self.old_cont_area) / self.cont_factor, 0, None)

        nav_reward = 0.2 if sum(self.nav_pixel) == 0 else 0

        total_reward = fruit_reward + map_reward + contour_reward - self.step_dicount + nav_reward
        print(f'Total: {total_reward}, Nav reward: {nav_reward}, Map reward: {map_reward}, contour: {contour_reward}')
        return total_reward

    def start_ros_simulation(self):
        print("Starting navigation subprocess:")
        self.nav_proc = Popen("exec ros2 run agrobot_rl world_reset_node.py",
                              shell=True,
                              preexec_fn=os.setsid,
                              stdout=PIPE
                              )
        print("Starting simulation subprocess:")
        self.sim_proc = Popen(f"exec ros2 launch agrobot_rl rl_env_gazebo.launch.py "
                              f"headless:={str(self.headless)} use_rviz:={str(self.use_rviz)} "
                              f"rviz_config:={self.rviz_config}",
                              shell=True,
                              preexec_fn=os.setsid,
                              stdout=PIPE
                              )
        while not self.running:
            running_nodes = run('ros2 node list',
                                shell=True, stdout=PIPE).stdout.decode("utf-8")
            if all(x in running_nodes for x in self.req_nodes):
                self.running = True
                break
            else:
                print("Not all nodes are running, waiting...")
            sleep(1.5)

        self.robot_spawned = 'agrobot_base' in run('ros2 service call /get_model_list gazebo_msgs/srv/GetModelList',
                                                   shell=True, stdout=PIPE).stdout.decode("utf-8")
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

    def check_contour(self):
        image = self.state
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Blurring the image in order to close small gaps in the contour:
        img_dill = cv2.dilate(img_gray, kernel)

        # Converting to black/white
        ret, im = cv2.threshold(img_dill, 127, 255, cv2.THRESH_BINARY_INV)

        # Finding contours
        contours, hierarchy = cv2.findContours(im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check if there is a closed contour containing another contour (which has a significant area):
        has_inner = [x > 0 for x in [el[3] for el in hierarchy[0]]]
        area = [cv2.contourArea(cnt) for cnt in contours]
        cont_area = np.sum(area[1:])
        done = any([x > 100 for x in list(compress(area, has_inner))])

        return done, cont_area

    def check_fruit(self):
        '''
        Function checks the number of fruits on the new map and
        compares it with the number of fruit on the previous map.
        updates the nr of fruits- and the new nr of fruits variables
        returns True if all fruit has been detected
        '''
        fruit_found = 0
        # TODO: count number of fruit dots in observation space
        # these will be single colour, so check contours with opencv in one of
        # the three colour channels and count the number of contours with area <= x
        fruit_found = 0  # total number of fruit found on the map
        self.new_fruit = fruit_found - self.nr_fruit
        self.nr_fruit = fruit_found

        return fruit_found == self.tot_fruit

    def close(self):
        self.kill_ros_simulation()

    def destroy(self):
        self.kill_ros_simulation()


def main():
    try:
        gym.envs.register(id="agrobot-v0", entry_point="gym_agrobot.envs:AgrobotExploreEnv")
        env = gym.make("agrobot-v0")
        now = datetime.now().strftime("%Y%m%d_%H%M")
        log_dir = str(Path.home()) + '/agrobot_ws/src/agrobot/agrobot_rl/tmp/log'
        tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)
        model = PPO('CnnPolicy', env, learning_rate=1e-3, verbose=1, tensorboard_log='log_dir')
        # check_env(env)
        model.learn(total_timesteps=100, tb_log_name='run_' + now)
        model_path = f"/home/mtn/agrobot_ws/src/agrobot/agrobot_rl/tmp/models/{now}"
        model.save(model_path)

        del model

        model = DQN.load(model_path)

        mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
        print(f'mean reward: {mean_reward}, std reward: {std_reward}')
    except Exception as ex:
        print(f'Caught exception: {ex}')
    finally:
        env.close()


if __name__ == "__main__":
    main()
