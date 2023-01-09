import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
#from communication_control_module.animation import Motion_management
import math

#fig,ax=plt.subplots()
interrupt_flag=0

plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体 # 解决中文乱码问题（仅针对windows电脑）
plt.rcParams['axes.unicode_minus'] = False  # 解决负号无法显示问题
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
        plt.plot(motion_mana.follow_cals[i].uav_follow.uav[-1].x,
                 motion_mana.follow_cals[i].uav_follow.uav[-1].y,"r.")
    for i,txt in enumerate(n):
        plt.annotate(txt,(motion_mana.follow_cals[i].uav_follow.uav[-1].x,
                          motion_mana.follow_cals[i].uav_follow.uav[-1].y))
    plt.arrow(motion_mana.motion_cal.uav_main.uav[-1].x, motion_mana.motion_cal.uav_main.uav[-1].y,
              2 * math.cos(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              2 * math.sin(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              head_length=1.5 * 0.1 * 6, head_width=0.1 * 4) #画箭头
    plt.grid(True)
    plt.xlim([-10, 50])
    plt.ylim([-10, 50])
    plt.pause(0.0001)

def show_detection_animation(follow_locus,motion_mana):
    plt.cla()
    plt.plot(motion_mana.cals[0].map.object[0], motion_mana.cals[0].map.object[1], "r*", label='Object')

    xs = [[] for i in range(len(follow_locus))]
    ys = [[] for i in range(len(follow_locus))]

    n = np.arange(len(follow_locus))
    for i in range(len(follow_locus)):
        plt.plot(motion_mana.cals[i].uav_main.uav[-1].x,
                 motion_mana.cals[i].uav_main.uav[-1].y, "r.")
        plt.gca().add_artist(
            plt.Circle((follow_locus[i][0].x, follow_locus[i][0].y), radius=4.0, edgecolor='b', alpha=0.25))
    for i, txt in enumerate(n):
        plt.annotate(txt, (motion_mana.cals[i].uav_main.uav[-1].x,
                           motion_mana.cals[i].uav_main.uav[-1].y))
    plt.grid(True)
    plt.legend(loc='lower right')
    plt.xlim([-10, 50])
    plt.ylim([-10, 50])
    plt.pause(0.00001)

#显示单机避障效果
def show_avoid_obstacles(locus, motion_mana):
    plt.cla()
    plt.scatter(motion_mana.motion_cal.map.obstacle[:, 0], motion_mana.motion_cal.map.obstacle[:, 1], s=5)  # 画障碍物
    plt.plot(motion_mana.motion_cal.goal[0], motion_mana.motion_cal.goal[1], "rv")  # 画目标
    # 主机坐标
    x = []
    y = []
    plt.plot(locus[0].x, locus[0].y, "k.")  # 画主机
    plt.arrow(motion_mana.motion_cal.uav_main.uav[-1].x, motion_mana.motion_cal.uav_main.uav[-1].y,
              2 * math.cos(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              2 * math.sin(motion_mana.motion_cal.uav_main.uav[-1].yaw),
              head_length=1.5 * 0.1 * 6, head_width=0.1 * 4)  # 画箭头
    plt.grid(True)
    plt.xlim([-10, 50])
    plt.ylim([-10, 50])
    plt.pause(0.0001)

