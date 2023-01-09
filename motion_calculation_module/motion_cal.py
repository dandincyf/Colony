from uav_module.uav import Uav,State
from communication_control_module.map import Map
import numpy as np
import matplotlib.pyplot as plt
import math

class Motion_Cal(object):
    def __init__(self,uav,map):
        self.uav_main = uav
        self.map = map
        self.predicttime = 1
        self.to_goal_coeff = 1.0
        self.velocity_coeff = 1.0
        self.dt = 0.1
        self.goal = np.array([0,0])
        self.detection_radius = 4.0

    # ------------------------------------
    # 运动函数
    def motion(self, velocity, yawrate):
        temp_state = State(self.uav_main.uav[-1].time + self.dt,
                           self.uav_main.uav[-1].x + velocity * math.cos(self.uav_main.uav[-1].yaw) * self.dt,
                           self.uav_main.uav[-1].y + velocity * math.sin(self.uav_main.uav[-1].yaw) * self.dt,
                           velocity,
                           self.uav_main.uav[-1].yaw + yawrate * self.dt,
                           yawrate,
                           (yawrate - self.uav_main.uav[-1].yawrate) / self.dt)
        self.uav_main.uav.append(temp_state)
        return temp_state

    # -----------
    # 动态窗口，返回速度范围、角速度范围
    def motion_windows(self):
        current_velocity = self.uav_main.uav[-1].velocity
        current_yawrate = self.uav_main.uav[-1].yawrate
        maxvelocity = current_velocity + self.uav_main.maxlinearacc * self.predicttime
        minvelocity = current_velocity - self.uav_main.maxlinearacc * self.predicttime
        maxyawrate = current_yawrate + self.uav_main.maxdyawrate * self.predicttime
        minyawrate = current_yawrate - self.uav_main.maxdyawrate * self.predicttime

        return np.array([minvelocity, maxvelocity, minyawrate, maxyawrate])

    # ------------------------------------
    # 三项成本函数的定义-------------------
    def cost_goal(self, locus):
        return distance(np.array([locus[-1].x, locus[-1].y]), self.goal) * self.to_goal_coeff

    def cost_velocity(self, locus):
        return (self.uav_main.maxvelocity - locus[-1].velocity) * self.velocity_coeff

    #避障策略，只考虑感受野范围内的障碍物
    def cost_obstacle(self, locus):
        dis = []
        for i in locus:
            for ii in self.map.obstacle:
                if distance(np.array([i.x, i.y]), ii) < 4:
                    dis.append(distance(np.array([i.x, i.y]), ii))
        dis_np = np.array(dis)
        if len(dis_np) == 0:
            return 0.0
        else:
            return 1.0 / np.min(dis_np)

    def cost_total(self, locus):
        return self.cost_goal(locus) + self.cost_velocity(locus) + self.cost_obstacle(locus)

    # -----------------------------------
    # 遍历轨迹
    def search_for_best_uv(self):
        windows = self.motion_windows()
        best_uv = np.array([0, 0])
        currentcost = np.inf
        inituav = self.uav_main.uav[:]
        best_locus = []

        for i in np.arange(windows[0], windows[1], self.uav_main.velocityres):  # 遍历速度
            for ii in np.arange(windows[2], windows[3], self.uav_main.yawrateres):  # 遍历角速度
                locus = []
                t = 0

                while (t <= self.predicttime):  # 预测3s后的位置
                    locus.append(self.motion(i, ii))
                    t = t + self.dt
                newcost = self.cost_total(locus)  # 算该预测位置的损失
                if currentcost > newcost:  # 找一个损失最小的预测点，得到最优位置，最优速度、角速度
                    currentcost = newcost
                    best_uv = [i, ii]
                    best_locus = locus[:]
                    best_locus[0].cost = currentcost
                self.uav_main.uav = inituav[:]

        return best_locus, best_uv, currentcost

    # -------------------------------------------------------------------
    # 安全检测
    def safedetect(self):
        positionx = self.uav_main.uav[-1].x
        positiony = self.uav_main.uav[-1].y
        position = np.array([positionx, positiony])
        for i in self.map.obstacle:
            if distance(position, i) <= self.uav_main.saferadius:
                print("over!")
                time.sleep(100000)

    #更新goal坐标
    def update_goal(self,goal):
        self.goal = goal
        
    def random_search(self):
        track_points = []
        for point in track_points:
            self.goal.append(point)
        # alg()
    
    def get_obj(self):
        if distance(self.map.object, self.loc) <= self.detection_radius:
            return self.map.nearest_point()
        else:
            return False

class Follow_Cal(object):
    def __init__(self,uav,map):
        self.uav_follow = uav
        self.map = map
        self.predicttime = 2
        self.to_goal_coeff = 100.0
        self.velocity_coeff = 1.0
        self.dt = 1
        self.goal = np.array([0,0])

    # ------------------------------------
    # 运动函数
    def motion(self, velocity, yawrate):
        temp_state = State(self.uav_follow.uav[-1].time + self.dt,
                           self.uav_follow.uav[-1].x + velocity * math.cos(self.uav_follow.uav[-1].yaw) * self.dt,
                           self.uav_follow.uav[-1].y + velocity * math.sin(self.uav_follow.uav[-1].yaw) * self.dt,
                           velocity,
                           self.uav_follow.uav[-1].yaw + yawrate * self.dt,
                           yawrate,
                           0)
        self.uav_follow.uav.append(temp_state)
        return temp_state

    # ------------------------------------
    # 动态窗口定义-------------------------
    def motion_windows(self):
        current_velocity = self.uav_follow.uav[-1].velocity
        current_yawrate = self.uav_follow.uav[-1].yawrate
        maxvelocity = current_velocity + self.uav_follow.maxlinearacc * self.predicttime
        minvelocity = current_velocity - self.uav_follow.maxlinearacc * self.predicttime
        maxyawrate = current_yawrate + self.uav_follow.maxdyawrate * self.predicttime
        minyawrate = current_yawrate - self.uav_follow.maxdyawrate * self.predicttime

        return np.array([minvelocity, maxvelocity, minyawrate, maxyawrate])

    # ------------------------------------
    # 成本函数
    def cost_goal(self, locus):
        return distance(np.array([locus[-1].x, locus[-1].y]), self.goal) * self.to_goal_coeff

    def cost_velocity(self, locus):
        return (self.uav_follow.maxvelocity - locus[-1].velocity) * self.velocity_coeff

    def cost_obstacle(self, locus):
        dis = []
        for i in locus:
            for ii in self.map.obstacle:
                dis.append(distance(np.array([i.x, i.y]), ii))
        dis_np = np.array(dis)
        return 1.0 / np.min(dis_np)

    def cost_total(self, locus):
        return self.cost_goal(locus) + self.cost_velocity(locus) * 0 + self.cost_obstacle(locus) * 0

    # -----------------------------------
    # 遍历轨迹
    def search_for_best_uv(self):
        windows = self.motion_windows()
        best_uv = np.array([0, 0])
        currentcost = np.inf
        inituav = self.uav_follow.uav[:]
        best_locus = []

        for i in np.arange(windows[0], windows[1], self.uav_follow.velocityres):  # 遍历速度
            for ii in np.arange(windows[2], windows[3], self.uav_follow.yawrateres):  # 遍历角速度
                locus = []
                t = 0

                while (t <= self.predicttime):  # 预测3s后的位置
                    locus.append(self.motion(i, ii))
                    t = t + self.dt
                newcost = self.cost_total(locus)  # 算该预测位置的损失
                if currentcost > newcost:  # 找一个损失最小的预测点，得到最优位置，最优速度、角速度
                    currentcost = newcost
                    best_uv = [i, ii]
                    best_locus = locus[:]
                    best_locus[0].cost = currentcost
                self.uav_follow.uav = inituav[:]

        return best_locus, best_uv, currentcost

    # -------------------------------------------------------------------
    # 安全检测
    def safedetect(self):
        positionx = self.uav_follow.uav[-1].x
        positiony = self.uav_follow.uav[-1].y
        position = np.array([positionx, positiony])
        for i in self.map.obstacle:
            if distance(position, i) <= self.uav_follow.saferadius:
                print("over!")
                time.sleep(100000)

    # 更新goal坐标
    def update_goal(self, goal):
        self.goal = goal

# -----------------------------------------------------------------
def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

