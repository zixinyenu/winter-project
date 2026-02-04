import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from .Astar import *

def run_astar(start, goal):
    start_x = start[0]
    start_y = start[1]
    goal_x = goal[0]
    goal_y = goal[1]

    solution_path = a_star(
        (start_x, start_y), (goal_x, goal_y), online=False
    )

    xlist = []
    ylist = []
    for node in solution_path:
        x, y = ij2xy(node[0], node[1])
        xlist.append(x)
        ylist.append(y)

    # fig, ax = plt.subplots()
    # plt.plot(xlist, ylist, ".")
    # circle1 = plt.Circle((2.0, 2.0), 0.5, color='red', fill=True)
    # rec1 = Rectangle((-1.25, -4.5), 1.0, 1.0, color='red', fill=True)
    # circle2 = plt.Circle((-3.0, -2.0), 0.5, color='red', fill=True)
    # rec2 = Rectangle((-2.5, -0.25), 3.0, 0.5, color='red', fill=True)
    # rec3 = Rectangle((-4.0, 2.25), 2.0, 1.5, color='red', fill=True)
    # ax.add_patch(circle1)
    # ax.add_patch(rec1)
    # ax.add_patch(circle2)
    # ax.add_patch(rec2)
    # ax.add_patch(rec3)
    
    # # Plot obstacles
    # plt.show()

    return xlist, ylist
