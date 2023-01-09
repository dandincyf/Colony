#无人机信息及地图管理

import numpy as np

# 地图类
class Map(object):
    def __init__(self):
        self.assemble = np.array([
            [10, 10],
            [20.0, 20.0],
            [6.0, 16.0],
            [16.0, 6.0],
            [30.0, 30.0],
            [28.0, 26.0],
            [18.0, 22.0],
        ])
        self.assemble1 = np.array([
            [10, 10],
            [20.0, 20.0],
            [6.0, 7.0],
            [12.0, 8.0],
            [30.0, 30.0],
            [28.0, 26.0],
            [18.0, 22.0],
            [12.0, 15.0],
            [15.0, 15.0],
            [22.0, 18.0],
            [7.0, 9.0],
            [33.0, 30.0],
            [23.0, 24.0],
            [18.0, 25.0],
            [5.0, 5.0],
            [8.0, 3.0],
            [3.0, 8.0],
            [2.0, 2.0],
        ])
        self.locus = np.vstack(([self.assemble],))

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
        
    def initialobject(self):
        self.object = np.array([37,37])

    def multi_obstacles(self):
        self.obstacle = self.assemble1 #用在show_avoid_obstacles的障碍物设置中

