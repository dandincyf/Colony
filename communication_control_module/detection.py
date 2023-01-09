import motion_calculation_module.motion_cal as mc
import time
import matplotlib.pyplot as plt
import math
import numpy as np
from other_module import draw
from uav_module.uav import Uav,State
from communication_control_module.map import Map

class Info_management_detec(object): #无人机信息管理类
    def __init__(self):
        self.follow_num = 10
        self.uavs = [Uav(0.5,0.1) for i in range(self.follow_num)]
        self.map = Map()

class Motion_management_detec(object):#无人机运动管理类
    def __init__(self,info_mana):
        self.cals = []
        for i in range(info_mana.follow_num):
            self.cals.append(mc.Motion_Cal(info_mana.uavs[i],info_mana.map))

def object_detection_mission():
    info_mana = Info_management_detec()
    motion_mana = Motion_management_detec(info_mana)
    states = []
    for i in range(info_mana.follow_num):
            states.append(mc.State(0, i+1, 0, 0.2, math.pi / 2, 0, 0))

    goal_list = [[[-6.0 + 5*i, -6.0], [-6.0 + 5*i, 46.0], [-6.0 + 5*i, 46.0],
                  [-6.0 + 5*i, -6.0]] for i in range(info_mana.follow_num)]
    cur_goal = [0 for i in range(info_mana.follow_num)]
    for i in range(info_mana.follow_num):
        motion_mana.cals[i].goal = np.array(goal_list[i][0])
        motion_mana.cals[i].uav_main.initialState(states[i])
        motion_mana.cals[i].map.initialobject()
        motion_mana.cals[i].map.initialobstacle()

    costs = [0 for i in range(info_mana.follow_num)]
    best_uvs = [np.array([0, 0]) for i in range(info_mana.follow_num)]
    best_follow_locus = [0 for i in range(info_mana.follow_num)]
    obj_flag = 0
    obstacles = np.zeros((10, 2))  # 存放所有无人机作为障碍物
    fig = plt.figure(figsize=(6, 6))

    for i in range(1000):
        if obj_flag == 1:
            break
        print('i=',i)
        time_begin = time.time()
        obstacles = np.zeros((10, 2))

        for j in range(info_mana.follow_num):
            if mc.distance(
                    np.array([motion_mana.cals[j].uav_main.uav[-1].x, motion_mana.cals[j].uav_main.uav[-1].y]),
                    motion_mana.cals[j].map.object) < motion_mana.cals[j].detection_radius:
                print("Object Found!")
                obj_flag = 1
                break
            if mc.distance(
                    np.array([motion_mana.cals[j].uav_main.uav[-1].x, motion_mana.cals[j].uav_main.uav[-1].y]),
                    motion_mana.cals[j].goal) < motion_mana.cals[j].uav_main.saferadius:
                motion_mana.cals[j].update_goal(goal_list[j][cur_goal[j]+1])
                cur_goal[j] = cur_goal[j] + 1

            best_follow_locus[j], best_uvs[j], costs[j] = motion_mana.cals[j].search_for_best_uv()
            obstacles[j, :] = [best_follow_locus[j][0].x, best_follow_locus[j][0].y]

        # 每架无人机执行自带的避障算法，对协同算法得到的操作指令进行调整
        for j in range(info_mana.follow_num):
            motion_mana.cals[j].uav_main.obstacle_avoid(obstacles, j)
            obstacles[j, :] = [motion_mana.cals[j].uav_main.uav[-1].x, motion_mana.cals[j].uav_main.uav[-1].y]

        newstates = []
        for j in range(info_mana.follow_num):
            newstates.append(motion_mana.cals[j].motion(best_uvs[j][0], best_uvs[j][1]))
            motion_mana.cals[j].uav_main.uav[-1].cost = costs[j]
        draw.show_detection_animation(best_follow_locus, motion_mana)
        time_end = time.time()

if __name__ == '__main__':
    object_detection_mission()