#无人机定义相关类
import math
import numpy as np
import time
import matplotlib.pyplot as plt

#无人的状态类，记录无人机每时刻的状态
class State(object):
    def __init__(self, time, x, y, velocity, yaw, yawrate, dyawrate, cost):
        self.time = time
        self.x = x
        self.y = y
        self.velocity = velocity
        self.yaw = yaw
        self.yawrate = yawrate
        self.dyawrate = dyawrate
        self.cost = cost
        self.saferadius = 0.5
        self.safeflag = True

class Uav(object):
    def __init__(self,maxv,maxa):
        self.uav = []
        self.maxvelocity = maxv # 2, 4
        self.minvelocity = 0
        self.maxlinearacc = maxa # 0.1, 0.5
        self.maxdyawrate = 40 * math.pi / 180
        self.velocityres = 0.01
        self.yawrateres = 0.5 * math.pi / 180
        self.saferadius = 0.5
        self.arrive = False

    # 初始化函数----------------------------
    def initialState(self, state):
        self.uav.append(state)


