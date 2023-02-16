import copy
import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np

try:
    import dubins_path_planning
    import dubins_path_problem
    from rrt_planner import rrt_planner
    from rrt_star_planner import rrt_star_planner
except ImportError:
    raise

random.seed(101)
total_len=0
obstacleList = [
    (5, 5, 1),
    (3, 6, 2),
    (3, 8, 2),
    (3, 10, 2),
    (7, 5, 2),
    (9, 5, 2)
]  # [x,y,size(radius)]
# Set Initial parameters
start = [0.0, 0.0, np.deg2rad(-50.0)]
goal = [10.0, 10.0, np.deg2rad(50.0)]



for i in range(1000):
    print('epoch:',i+1)
    rrt_dubins = dubins_path_problem.RRT_dubins_problem(start = start, goal = goal, \
                                                        obstacle_list = obstacleList, \
                                                        map_area = [-2.0, 15.0, -2.0, 15.0], \
                                                        max_iter=10000)

    path_nodes= rrt_dubins.rrt_planning(display_map=False)
    total_len+= path_nodes[-1].cost

mean_rrt=total_len/1000


total_len=0

for i in range(1000):
    print('epoch:',i+1)
    rrt_dubins = dubins_path_problem.RRT_dubins_problem(start = start, goal = goal, \
                                                        obstacle_list = obstacleList, \
                                                        map_area = [-2.0, 15.0, -2.0, 15.0], \
                                                        max_iter=10000)

    path_nodes= rrt_dubins.rrt_star_planning(display_map=False)
    total_len+= path_nodes[-1].cost

mean_rrt_star=total_len/1000

fig = plt.figure()
#ax = fig.add_axes([0,0,1,1])
algo = ['RRT', 'RRT*']
cost = [mean_rrt,mean_rrt_star]
plt.ylabel('Mean Path Cost')
plt.title('Comparison between RRT and RRT* over 1000 iter')
plt.xlabel('Algorithms')
plt.bar(algo,cost)

plt.show()
