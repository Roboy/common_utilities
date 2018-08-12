#!//bin/env python
import numpy
import unittest
import openravepy
import subprocess
import os
import sys

# Find the bh_280 URDF file
from catkin.find_in_workspaces import find_in_workspaces

env = openravepy.Environment()
urdf_module = openravepy.RaveCreateModule(env, 'urdf')
with env:
    name = urdf_module.SendCommand('load /home/letrend/workspace/roboy_middleware/src/roboy_models/roboy_xylophone_left_arm/model.urdf /home/letrend/workspace/roboy_middleware/src/roboy_xylophone_hand_left_moveit_config/config/roboy_xylophone_left_arm.srdf')
    body = env.GetRobot(name)
    env.Save('/home/letrend/workspace/roboy_middleware/src/roboy_models/roboy_xylophone_left_arm/roboy_xylophone_left_arm.xml')
    print('done')