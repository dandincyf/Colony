#无人机定义相关类
import math
import numpy as np
import time
import matplotlib.pyplot as plt

#无人的状态类，记录无人机每时刻的状态
class State(object):
    def __init__(self, time, x, y, velocity, yaw, yawrate, cost):
        self.time = time
        self.x = x #位置
        self.y = y
        self.velocity = velocity #速度
        self.yaw = yaw #偏航角
        self.yawrate = yawrate #角速度
        self.cost = cost #损失
        self.safeflag = True

class Uav(object):
    def __init__(self,maxv,maxa):
        self.uav = [] #存放每一时刻的状态State
        self.maxvelocity = maxv #允许的最大速度，默认 2, 4
        self.maxlinearacc = maxa #允许的最大加速度，默认 0.1, 0.5
        self.maxdyawrate = 40 * math.pi / 180 #允许的最大角加速度
        self.velocityres = 0.02 #遍历速度时的增量
        self.yawrateres = 0.5 * math.pi / 180 #遍历角速度时的增量
        self.saferadius = 0.5 #安全距离，判定是否撞机
        self.arrive = False

    # 初始化函数----------------------------
    def initialState(self, state):
        self.uav.append(state) #添加初始State


