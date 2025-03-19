import panda_py
from panda_py import libfranka
import logging
from panda_py import constants
import numpy as np
from matplotlib import pyplot as plt

# Panda hostname/IP and Desk login information of your robot
hostname = '192.168.1.100'

# panda-py is chatty, activate information log level

# 获取IP
# Panda hostname/IP and Desk login information of your robot
hostname = '192.168.1.100'

# panda-py is chatty, activate information log level
import logging
logging.basicConfig(level=logging.INFO)


# 初始化
panda = panda_py.Panda(hostname)
gripper = libfranka.Gripper(hostname)

# 让 Panda 机械臂移动到预定义的初始位置
panda.move_to_start()

# 获取 Panda 机械臂末端执行器（End Effector）的当前位姿（position + orientation）
pose = panda.get_pose()

# get_pose 会生成一个矩阵， 4*4 矩阵表示从机器人基础到末端执行器的其次变换。
# 索引2，3表示第三列和第四列。即Z坐标。
#z 中的位置由 0.1m inverse kinematics 函数降低并传递给 inverse kinematics 函数
# -————————————————
# pose[2,3] -= .1 # 向下移动
# q = panda_py.ik(pose)
# # 调用 move_to_joint_position 生成运动；从当前位置到期望的位置。
# panda.move_to_joint_position(q)
# print ("pose: ", pose) # 打印出来 当前的位姿
# pose[2,3] += .1 # 向上移动
# q = panda_py.ik(pose)
# panda.move_to_joint_position(q)

# pose[1,2] -= .1 # 向左倾斜
# q = panda_py.ik(pose)
# panda.move_to_joint_position(q)
# panda.move_to_start()

# pose[1,2] += .1 # 向右倾斜
# q = panda_py.ik(pose)
# panda.move_to_joint_position(q)
# panda.move_to_start()


# ————————————————inline examples 控制夹爪

# print(panda.get_state())
# print(panda.get_model())
# gripper.grasp(0, 0.2, 10, 0.04, 0.04)
# gripper.move(0.08, 0.2)


#  ——————————————
# 创建两个末端执行器的位姿，T0和T1，起始姿势向左和向右移动0.25M
# T[0, 3] += 0.1  # X 轴方向移动 10 cm
# T[1, 3] += -0.05 # Y 轴方向移动 -5 cm
# T[2, 3] += 0.2  # Z 轴方向移动 20 cm

T_0 = panda_py.fk(constants.JOINT_POSITION_START)
# 在Y轴上进行移动
T_0[1, 3] = 0.1
T_1 = T_0.copy()
T_1[1, 3] = -0.1




# T_3 = T_0.copy()
# T_3[0, 3] = -0.1

panda.move_to_pose(T_0)
panda.enable_logging(40000)
panda.move_to_pose(T_1)
# panda.move_to_pose(T_1, speed_factor=0.01, stiffness=2 * np.array([600, 600, 600, 600, 250, 150, 50]))
panda.disable_logging()
log = panda.get_log()
print("ni1111111111111111111111111")
panda.move_to_pose(T_0)

panda.move_to_start()
T_2 = panda_py.fk(constants.JOINT_POSITION_START)
T_2[0, 3] = 0.4
panda.move_to_pose(T_2)

#panda.move_to_pose(T_3)
#panda.enable_logging(10000)
print("ni11111111111111111111111111")
# panda.move_to_joint_position(panda_py.ik(T_1))
# panda.move_to_joint_position(panda_py.ik(T_0))


print("ni111111111111111111111111111")
panda.move_to_start()
panda.disable_logging()

joint_log = panda.get_log()
