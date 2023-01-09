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
        self.velocityres = 0.05 #遍历速度时的增量
        self.yawrateres = 0.5 * math.pi / 180 #遍历角速度时的增量
        self.saferadius = 0.4 #安全距离，判定是否撞机
        self.arrive = False
        self.perception_range = 3  # 感受野，能感受到这个范围内的障碍物

    # 初始化函数----------------------------
    def initialState(self, state):
        self.uav.append(state) #添加初始State

    # 无人机自动避障函数
    def obstacle_avoid(self, obstacle_coor, j):
        obstacle_coor = np.delete(obstacle_coor,obstacle_coor.shape[0]-10+j,axis=0) #排除自己
        for i in obstacle_coor: #靠近障碍物（包括其他无人机）就自动避开
            while(distance(np.array([self.uav[-1].x,self.uav[-1].y]),i)<0.3):
                if self.uav[-1].x<i[0]:
                    self.uav[-1].x -= 0.05
                else:
                    self.uav[-1].x += 0.05

def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
