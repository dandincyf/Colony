import motion_calculation_module.motion_cal as mc
import time
import matplotlib.pyplot as plt
import math
import numpy as np
from other_module import draw
from uav_module.uav import Uav, State
from communication_control_module.map import Map
#from ..motion_calculation_module import motion_cal as mc


class Motion_management(object):  # 无人机运动管理类
    def __init__(self, info_mana):
        self.mission = ['evolution', 'Detection',
                        'motion control']  # 任务类型：改变队形、搜寻目标、运动控制
        self.motion_cal = mc.Motion_Cal(
            info_mana.uav_main, info_mana.map)  # 协同运动计算对象，主机
        self.follow_cals = []
        # 协同运动计算对象，僚机
        for i in range(info_mana.follow_num):
            self.follow_cals.append(mc.Follow_Cal(
                info_mana.uavs_follow[i], info_mana.map))


class Info_management(object):  # 无人机信息管理类
    def __init__(self):
        self.uav_main = Uav(0.1, 0.1)
        self.follow_num = 9
        self.uavs_follow = []
        for i in range(self.follow_num):
            self.uavs_follow.append(Uav(2, 0.1))
        self.map = Map()

#if __name__ == '__main__':
def flyShow():
    info_mana = Info_management()  # 创建一个无人机信息管理对象
    motion_mana = Motion_management(info_mana)  # 创建一个运动管理对象
    motion_mana.mission = 'evolution'
    if motion_mana.mission == 'evolution':
        states = []
        # 设置10架无人机初始状态
        states.append(mc.State(0, 0, 0, 0.2, math.pi / 2, 0, 0))  # 主机
        for i in range(info_mana.follow_num):  # 僚机
            states.append(mc.State(0, i+1, 0, 0.2, math.pi / 2, 0, 0))
        # 主机参数初始化
        motion_mana.motion_cal.goal = np.array([35, 35])
        motion_mana.motion_cal.uav_main.initialState(states[0])
        motion_mana.motion_cal.map.initialobstacle()
        # 僚机参数初始化
        for i in range(info_mana.follow_num):
            motion_mana.follow_cals[i].uav_follow.initialState(states[i+1])
            motion_mana.follow_cals[i].map.initialobstacle()
        cost = 0
        best_uv = np.array([0, 0])
        costs = []
        best_uvs = []
        best_follow_locus = []
        for i in range(info_mana.follow_num):
            costs.append(0)
            best_uvs.append(0)
            best_follow_locus.append(0)
        obstacles = np.zeros((info_mana.map.obstacle.shape[0] + 10, 2))  # 存放所有无人机和障碍物坐标
        fig = plt.figure(figsize=(10, 8))

        # 可以改成while
        for i in range(1000):
            obstacles[0:info_mana.map.obstacle.shape[0], :] = info_mana.map.obstacle
            time_begin = time.time()
            best_locus, best_uv, cost = motion_mana.motion_cal.search_for_best_uv()  # 计算主机的最佳位置、最佳速度、成本
            print('i=', i)
            for j in range(info_mana.follow_num):  # 计算僚机的最佳位置、最佳速度、成本
                if i < 35:
                    motion_mana.follow_cals[j].update_goal(np.array([best_locus[0].x + 5 * math.cos((j+1) * 2 *
                        math.pi / (info_mana.follow_num)), best_locus[0].y + 5 * math.sin((j+1) *
                                                2 * math.pi / (info_mana.follow_num))]))  # 僚机的目标点
                elif i >= 35 and i < 70:
                    motion_mana.follow_cals[j].update_goal(np.array([best_locus[0].x + 2*(j+1),
                                                                     best_locus[0].y]))  # 僚机的目标点
                elif i >= 70:
                    motion_mana.follow_cals[j].update_goal(np.array([best_locus[0].x - 2*(j+1),
                                                                     best_locus[0].y - 2*(j+1)]))  # 僚机的目标点
                best_follow_locus[j], best_uvs[j], costs[j] = motion_mana.follow_cals[j].search_for_best_uv()
                obstacles[info_mana.map.obstacle.shape[0] + 1 + j, :] = \
                    (best_follow_locus[j][0].x, best_follow_locus[j][0].y)

            # 每架无人机执行自带的避障算法，对协同算法得到的操作指令进行调整
            for j in range(info_mana.follow_num):
                motion_mana.follow_cals[j].uav_follow.obstacle_avoid(obstacles, j)
                obstacles[info_mana.map.obstacle.shape[0] + 1 + j, :] = [motion_mana.follow_cals[j].
                                        uav_follow.uav[-1].x,motion_mana.follow_cals[j].uav_follow.uav[-1].y]

            newstate = motion_mana.motion_cal.motion(best_uv[0], best_uv[1])  # 更新主机状态
            motion_mana.motion_cal.uav_main.uav[-1].cost = cost  # 更新主机成本
            newstates = []
            for j in range(info_mana.follow_num):
                newstates.append(motion_mana.follow_cals[j].motion(best_uvs[j][0], best_uvs[j][1]))  # 更新僚机状态
                motion_mana.follow_cals[j].uav_follow.uav[-1].cost = costs[j] # 更新僚机成本

            draw.show_animation(best_locus, best_follow_locus, motion_mana)  # 绘制动画
            time_end = time.time()
            motion_mana.motion_cal.safedetect()  # 安全检测

            # 判断是否到达目标
            if mc.distance(
                    np.array([motion_mana.motion_cal.uav_main.uav[-1].x,
                             motion_mana.motion_cal.uav_main.uav[-1].y]),
                    motion_mana.motion_cal.goal) < motion_mana.motion_cal.uav_main.saferadius:
                print("Done!")
                break


#if __name__ == '__main__':
def avoid_obstacles_animation():
    info_mana = Info_management()  # 创建一个无人机信息管理对象
    motion_mana = Motion_management(info_mana)  # 创建一个运动管理对象
    state = mc.State(0, 0, 0, 0.2, math.pi / 2, 0, 0) # 主机
    # 主机参数初始化
    motion_mana.motion_cal.goal = np.array([33, 33])
    motion_mana.motion_cal.uav_main.initialState(state)
    motion_mana.motion_cal.map.multi_obstacles()

    cost = 0
    best_uv = np.array([0, 0])
    costs = []
    best_uvs = []
    fig = plt.figure(figsize=(10, 8))

    # 可以改成while
    for i in range(1000):
        best_locus, best_uv, cost = motion_mana.motion_cal.search_for_best_uv()  # 计算主机的最佳位置、最佳速度、成本
        print('i=', i)

        newstate = motion_mana.motion_cal.motion(best_uv[0], best_uv[1])  # 更新主机状态
        motion_mana.motion_cal.uav_main.uav[-1].cost = cost  # 更新主机成本

        draw.show_avoid_obstacles(best_locus, motion_mana)  # 绘制动画
        time_end = time.time()
        motion_mana.motion_cal.safedetect()  # 安全检测

        # 判断是否到达目标
        if mc.distance(
                np.array([motion_mana.motion_cal.uav_main.uav[-1].x,
                         motion_mana.motion_cal.uav_main.uav[-1].y]),
                motion_mana.motion_cal.goal) < motion_mana.motion_cal.uav_main.saferadius:
            print("Done!")
            break