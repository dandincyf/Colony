import motion_calculation_module.motion_cal as mc
import time
import matplotlib.pyplot as plt
import math
import numpy as np
from other_module import draw

class Motion_management(object):#无人机运动管理类
    def __init__(self):
        self.motion_cal = mc.Motion_Cal()#协同运动计算对象，主机
        self.follow_cals = []
        #协同运动计算对象，僚机
        for i in range(9):
            self.follow_cals.append(mc.Follow_Cal())

if __name__ == '__main__':
    motion_mana = Motion_management();#创建一个运动管理对象
    states = []
    #设置10架无人机初始状态
    for i in range(10):
        states.append(mc.State(0, 0, 0, 0.2, math.pi / 2, 0, 0, 0))
    #主机参数初始化
    motion_mana.motion_cal.uav_main.initialState(states[0])
    motion_mana.motion_cal.map.initialgoal(np.array([35, 35]))
    motion_mana.motion_cal.map.initialmap(motion_mana.motion_cal.map)
    #僚机参数初始化
    for i in range(9):
        motion_mana.follow_cals[i].uav_follow.initialState(states[i+1])
        motion_mana.follow_cals[i].map.initialmap(motion_mana.follow_cals[i].map)
    cost = 0
    best_uv = np.array([0, 0])
    costs = []
    best_uvs = []
    best_follow_locus = []
    for i in range(9):
        costs.append(0)
        best_uvs.append(0)
        best_follow_locus.append(0)

    # 可以改成while
    for i in range(1000):
        time_begin = time.time()
        best_locus, best_uv, cost = motion_mana.motion_cal.search_for_best_uv()
        for j in range(9):
            motion_mana.follow_cals[j].map.initialgoal(np.array([best_locus[0].x+3*math.cos(j*2*math.pi/8),
                                                                 best_locus[0].y+3*math.sin(j*2*math.pi/8)]))#僚机的目标点
            temp_locus, best_uvs[j], costs[j] = motion_mana.follow_cals[j].search_for_best_uv()
            best_follow_locus[j] = temp_locus

        draw.show_animation(best_locus, best_follow_locus, motion_mana)
        newstate = motion_mana.motion_cal.motion(best_uv[0], best_uv[1])
        motion_mana.motion_cal.uav_main.uav[-1].cost = cost
        newstates = []
        for j in range(9):
            newstates.append(motion_mana.follow_cals[j].motion(best_uvs[j][0],best_uvs[j][1]))
            motion_mana.follow_cals[j].uav_follow.uav[-1].cost = costs[j]

        time_end = time.time()
        # obstacle.update()
        # motion_mana.motion_cal.map.initialmap(motion_mana.motion_cal.map)
        motion_mana.motion_cal.safedetect()

        if mc.distance(np.array([motion_mana.motion_cal.uav_main.uav[-1].x, motion_mana.motion_cal.uav_main.uav[-1].y]),
                       motion_mana.motion_cal.map.goal) < motion_mana.motion_cal.uav_main.saferadius:
            print("Done!")
            break

    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(15, 10))
    ax[0].scatter(motion_mana.motion_cal.map.obstacle[:, 0], motion_mana.motion_cal.map.obstacle[:, 1], s=5)
    ax[0].plot(motion_mana.motion_cal.map.goal[0], motion_mana.motion_cal.map.goal[1], "ro")
    lx = []
    ly = []
    lt = []
    lc = []
    for i in motion_mana.motion_cal.uav_main.uav:
        lx.append(i.x)
        ly.append(i.y)
        lt.append(i.time)
        lc.append(i.cost)
    ax[0].plot(lx, ly)
    for i in range(17):
        locusx, locusy = motion_mana.motion_cal.map.returnlocus(i)
        ax[0].plot(locusx, locusy)

    plt.show()

