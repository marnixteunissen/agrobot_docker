#!/usr/bin/env python3

import gym
from gym import error, spaces, utils
from gym.utils import seeding

class AgrobotPickEnv(gym.Env):

    def __init__(self):
        # Defining the action- and observation spaces:
        bounds_x = []
        bounds_y = []
        self.action_space = spaces.Box()
