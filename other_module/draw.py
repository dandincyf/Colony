import numpy as np
import matplotlib.pyplot as plt
from communication_control_module.animation import Motion_management
import math

# 动图绘制
def show_animation(locus,follow_locus,motion_mana):
    plt.cla()
    plt.scatter(motion_mana.motion_cal.map.obstacle[:, 0], motion_mana.motion_cal.map.obstacle[:, 1], s=5) #画障碍物
    plt.plot(motion_mana.motion_cal.goal[0], motion_mana.motion_cal.goal[1], "rv") #画目标
    #主机坐标
    x = []
    y = []
    #僚机坐标
    xs = [[] for i in range(len(follow_locus))]
    ys = [[] for i in range(len(follow_locus))]
    plt.plot(locus[0].x, locus[0].y, "k.") #画主机
    n = np.arange(len(follow_locus))
    for i in range(len(follow_locus)): #画僚机1-9
        plt.plot(follow_locus[i][0].x,
                 follow_locus[i][0].y,"r.")
    for i,txt in enumerate(n):
        plt.annotate(txt,(follow_locus[i][0].x, follow_locus[i][0].y))
    plt.arrow(motion_mana.motion_cal.uav_main.uav[-1].x, motion_mana.motion_cal.uav_main.uav[-1].y,
              2 * math.cos(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              2 * math.sin(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              head_length=1.5 * 0.1 * 6, head_width=0.1 * 4) #画箭头
    plt.grid(True)
    plt.xlim([-10, 50])
    plt.ylim([-10, 50])
    plt.pause(0.0001)

