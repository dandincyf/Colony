import numpy as np
import matplotlib.pyplot as plt
from communication_control_module.animation import Motion_management
import math

# 动图绘制
def show_animation(locus,follow_locus,motion_mana):
    plt.cla()
    plt.scatter(motion_mana.motion_cal.map.obstacle[:, 0], motion_mana.motion_cal.map.obstacle[:, 1], s=5)#画障碍物
    plt.plot(motion_mana.motion_cal.map.goal[0], motion_mana.motion_cal.map.goal[1], "ro")#画主机目标
    #主机坐标
    x = []
    y = []
    #僚机坐标
    xs = [[] for i in range(9)]
    ys = [[] for i in range(9)]
    # for i in locus:#主机
    #     x.append(i.x)
    #     y.append(i.y)
    # plt.plot(x, y, "g-")
    plt.plot(motion_mana.motion_cal.uav_main.uav[-1].x, motion_mana.motion_cal.uav_main.uav[-1].y, "k.")
    follow_num = 0
    for i in follow_locus:#僚机1-9
        # for j in i:
        #     xs[follow_num].append(j.x)
        #     ys[follow_num].append(j.y)
        # plt.plot(xs[follow_num][:],ys[follow_num][:],"g-")
        plt.plot(motion_mana.follow_cals[follow_num].uav_follow.uav[-1].x,
                 motion_mana.follow_cals[follow_num].uav_follow.uav[-1].y,"r.")
        follow_num += 1
    plt.arrow(motion_mana.motion_cal.uav_main.uav[-1].x, motion_mana.motion_cal.uav_main.uav[-1].y,
              2 * math.cos(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              2 * math.sin(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              head_length=1.5 * 0.1 * 6, head_width=0.1 * 4)
    plt.grid(True)
    plt.xlim([-10, motion_mana.motion_cal.map.goal[0] * 1.3])
    plt.ylim([-10, motion_mana.motion_cal.map.goal[1] * 1.3])
    plt.pause(0.0001)

