#无人机信息及地图管理

import numpy as np

# 地图类
class Map(object):
    def __init__(self):
        # self.assemble = 3 * np.array([
        #     [-1, -1],
        #     [0, 2],
        #     [2.0, 2.0],
        #     [2.0, 1.0],
        #     [3.0, 4.0],
        #     [2.5, 4.0],
        #     [2.0, 5.0],
        #     [5.0, 2.5],
        #     [5.0, 6.0],
        #     [5.0, 5.0],
        #     [6.0, 6.0],
        #     [7.0, 6.0],
        #     [10.0, 8.0],
        #     [10.0, 4.0],
        #     [8.0, 9.0],
        #     [7.0, 9.0],
        #     [12.0, 12.0]
        # ])
        self.assemble = 3 * np.array([
            [-1, -1],
        ])
        self.locus = np.vstack(([self.assemble],))
        # self.goal = np.array([0, 0])

    def update(self):#会动的障碍物
        for i in self.assemble:
            alpha = 2 * np.random.random() * np.pi #障碍物随机运动，0-360度
            i[0] = i[0] + 0.2 * np.cos(alpha) #在半径为0.2的圆上
            i[1] = i[1] + 0.2 * np.sin(alpha)
        self.locus = np.vstack((self.locus, [self.assemble]))

    def returnlocus(self, index): #
        lx = []
        ly = []
        for i in self.locus:
            lx.append(i[index][0])
            ly.append(i[index][1])
        return lx, ly

    def initialobstacle(self):
        self.obstacle = self.assemble

