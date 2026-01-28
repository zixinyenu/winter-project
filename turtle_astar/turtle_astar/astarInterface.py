import matplotlib.pyplot as plt
from Obstacles import *
from Astar import *

def run_astar():
    start_x = input("Please enter the x coordinate of the start position of Turtlebot3: \n")
    start_y = input("Please enter the y coordinate of the start position of Turtlebot3: \n")
    goal_x = input("Please enter the x coordinate of the goal position of Turtlebot3: \n")
    goal_y = input("Please enter the y coordinate of the goal position of Turtlebot3: \n")

    start_x = float(start_x)
    start_y = float(start_y)
    goal_x = float(goal_x)
    goal_y = float(goal_y)

    solution_path = a_star(
        (start_x, start_y), (goal_x, goal_y), online=False
    )

    return solution_path

    # obs_map = get_obstacle_map()
    # counter = 1
    # i_max = int(WIDTH*ONE_METER + 1)
    # j_max = int(LENGTH*ONE_METER + 1)
    # for i in range(0, i_max):
    #     for j in range(0, j_max):
    #         print(counter)
    #         counter += 1
    #         flag = obs_map[i][j]
    #         x, y = ij2xy(i, j)
    #         if flag == 1:
    #             plt.scatter(x, y, color='r')

    # xlist = []
    # ylist = []
    # for node in solution_path:
    #     x, y = ij2xy(node[0], node[1])
    #     xlist.append(x)
    #     ylist.append(y)
    # plt.plot(xlist, ylist)
    # plt.show()
