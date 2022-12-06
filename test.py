from uav_module.uav import State
import math

# state1 = State(0, 0, 0, 0.2, math.pi / 2, 0, 0, 0)
# a=[0 for i in range(9)]
# for i in range(9):
#     a[i] = [i,2*i,3*i]
# b=1

# a=[]
# while(1):
#     for i in range(9):
#         a.append(i)
#
#
# follow_cals = []
# #协同运动计算对象，僚机
# for i in range(9):
#     follow_cals.append(mc.Follow_Cal())
#
# for i in range(9):
#     a.append(i)
# a=[]
# b=1

a=[0 for i in range(9)]
for j in range(9):
    a[j]=math.cos(j * 2 * math.pi / 9)

b=1