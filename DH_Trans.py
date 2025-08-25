# import numpy as np

# # 定义一个函数： 单个关节的DH变换矩阵
# def dh_tansform(alpha, a, theta, d):
#     theta = np.deg2rad(theta)
#     alpha = np.deg2rad(alpha) # 角度转弧度

#     return np.array([
#         [np.cos(theta), -np.sin(theta), 0, a],
#         [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
#         [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
#         [0, 0, 0, 1]
#     ]) # T变换矩阵

# alpha = [90, -180, 90, 90, -90, 0]
# a = [0, 280, 0, 0, 0, 0]
# theta = []
# d = [115+128.3, 30, 20, 105+140, 28.5+28.5, 130+105]
# # 机械臂列表

# theta_HOME = [0, 345, 75, 0, 300, 0]
# theta_ZERO = [0, 0, 0, 0, 0, 0]
# theta_RETRACT = [357, 21, 150, 272, 320, 273]
# theta_PACKAGING = [270, 148, 148, 270, 140, 0]
# theta_PICK = [20.5, 313.5, 100, 265.5, 327, 57]
# # 待测theta值

# def forward_kinematics(thetas):
#     """
#     输入:thetas 是长度为6的关节角列表
#     输出:末端到基座的4x4变换矩阵
#     """
#     T = np.eye(4) #初始化为单位矩阵
#     for i in range(6):
#         T = np.dot(T, dh_tansform(alpha[i], a[i], thetas[i], d[i]))
#     return T

# T_home = forward_kinematics(theta_ZERO)
# print(np.round(T_home,3)) #保留3位小数

import numpy as np

# 定义一个函数： 单个关节的DH变换矩阵
def dh_tansform(alpha, a, theta, d):
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha) # 角度转弧度

    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
        [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
        [0, 0, 0, 1]
    ]) # T变换矩阵

alpha = [0, 90, -180, 90, 90, -90, 0]
a = [0, 0, 280, 0, 0, 0, 0]
theta = []
d = [128.3, -30, 0, -140, -28.5, -105, -130]
# 机械臂列表

theta_HOME = [0, 345, 75, 0, 300, 0, 0]
theta_ZERO = [0, 0, 0, 0, 0, 0, 0]
theta_RETRACT = [357, 21, 150, 272, 320, 273, 0]
theta_PACKAGING = [270, 148, 148, 270, 140, 0, 0]
theta_PICK = [20.5, 313.5, 100, 265.5, 327, 57, 0]
# 待测theta值

def forward_kinematics(thetas):
    """
    输入:thetas 是长度为6的关节角列表
    输出:末端到基座的4x4变换矩阵
    """
    T = np.eye(4) #初始化为单位矩阵
    for i in range(7):
        T = np.dot(T, dh_tansform(alpha[i], a[i], thetas[i], d[i]))
    return T

T_home = forward_kinematics(theta_ZERO)
print(np.round(T_home,3)) #保留3位小数

