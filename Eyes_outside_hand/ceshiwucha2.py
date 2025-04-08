import panda_py
from panda_py import libfranka
import logging
from panda_py import constants
import numpy as np
from matplotlib import pyplot as plt

# 初始化机械臂和夹爪
hostname = '192.168.1.100'
panda = panda_py.Panda(hostname)
gripper = libfranka.Gripper(hostname)

panda.move_to_start()
gripper.move(0.0, 0.2)
pose_start = panda.get_pose()



posebai1 = np.array(
[[ 1,  0, 0,  0.47961817],
 [ 0, -1,  0 , 0.20020294],
 [ 0, 0, -1,  0.023],
 [ 0., 0., 0., 1. ]]
)

posebai = np.array(
[[ 1,  0, 0,  0.47961817],
 [ 0, -1,  0 ,0.20020294],
 [ 0, 0, -1,  0.03],
 [ 0., 0., 0., 1. ]]
)
panda.move_to_pose(posebai)
panda.move_to_pose(posebai)
panda.move_to_pose(posebai)

panda.move_to_pose(posebai1)
panda.move_to_pose(posebai1)
panda.move_to_pose(posebai1)
