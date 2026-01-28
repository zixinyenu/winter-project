import numpy as np
import matplotlib.pyplot as plt

LENGTH = 12.0
WIDTH = 12.0
ONE_METER = 100
# TODO Chnage URDF later
collision_radius = 0.11

def circle_bumped(
        turtle_pos, 
        turtle_r, 
        circle_pos, 
        circle_r
):
    xt = turtle_pos[0]
    yt = turtle_pos[1]
    xo = circle_pos[0]
    yo = circle_pos[1]
    if (xt-xo)**2 + (yt-yo)**2 <= (turtle_r + circle_r)**2:
        return True
    else:
        return False

def rectangle_bumped(
        turtle_pos, 
        turtle_r, 
        northwestern_pos, 
        southeastern_pos
):
    xt = turtle_pos[0]
    yt = turtle_pos[1]
    xleft = northwestern_pos[0]
    yup = northwestern_pos[1]
    xright = southeastern_pos[0]
    ybottom = southeastern_pos[1]

    if xt + turtle_r >= xleft and xt - turtle_r <= xright and \
       yt - turtle_r <= yup and yt + turtle_r >= ybottom:
        return True
    else:
        return False

def boundary_bumped(
        turtle_pos, 
        turtle_r, 
        northwestern_pos, 
        southeastern_pos
):
    xt = turtle_pos[0]
    yt = turtle_pos[1]
    xleft = northwestern_pos[0]
    yup = northwestern_pos[1]
    xright = southeastern_pos[0]
    ybottom = southeastern_pos[1]

    if xt + turtle_r <= xleft and xt - turtle_r >= xright and \
       yt - turtle_r >= yup and yt + turtle_r <= ybottom:
        return True
    else:
        return False

def obstacle_bumped(turtle_pos, turtle_r):
    # cylinder_obstacle_1
    cir_1_pos = [2.0, 2.0]
    cir_1_r = 0.5
    # rectangle_obstacle_1
    rec_1_nw = [-1.25, -3.5]
    rec_1_se = [-0.25, -4.5]
    # cylinder_obstacle_2
    cir_2_pos = [-3.0, -2.0]
    cir_2_r = 0.5
    # rectangle_obstacle_2
    rec_2_nw = [-2.5, 0.25]
    rec_2_se = [0.5, -0.25]
    # rectangle_obstacle_3
    rec_3_nw = [-4.0, 3.75]
    rec_3_se = [-2.0, 2.25]
    # fence
    rec_fence_nw = [-6.0, 6.0]
    rec_fence_se = [6.0, -6.0]

    flag1 = circle_bumped(
        turtle_pos, 
        turtle_r, 
        cir_1_pos, 
        cir_1_r
    )
    flag2 = rectangle_bumped(
        turtle_pos, 
        turtle_r, 
        rec_1_nw, 
        rec_1_se
    )
    flag3 = circle_bumped(
        turtle_pos, 
        turtle_r, 
        cir_2_pos, 
        cir_2_r
    )
    flag4 = rectangle_bumped(
        turtle_pos, 
        turtle_r, 
        rec_2_nw, 
        rec_2_se
    )
    flag5 = rectangle_bumped(
        turtle_pos, 
        turtle_r, 
        rec_3_nw, 
        rec_3_se
    )
    flag_fence = boundary_bumped(
        turtle_pos, 
        turtle_r, 
        rec_fence_nw, 
        rec_fence_se
    )

    return flag1 or flag2 or flag3 or flag4 or flag5 or flag_fence

def ij2xy(i, j, dx=-6.0, dy=-6.0, d=ONE_METER):
    x = j/d + dx
    y = i/d + dy
    return x, y

def xy2ij(x, y, dx=-6.0, dy=-6.0, d=ONE_METER):
    i = int((y - dy)*d)
    j = int((x - dx)*d)
    return i, j

def get_obstacle_map(length=12.0, width=12.0, d=ONE_METER):
    i_max = int(width*d + 1)
    j_max = int(length*d + 1)
    obstacle_list = np.zeros(shape=(i_max, j_max))

    for i in range(0, i_max):
        for j in range(0, j_max):
            x, y = ij2xy(i, j)
            turtle_pos = [x, y]
            flag = obstacle_bumped(turtle_pos, collision_radius)
            if flag:
                obstacle_list[i][j] = 1
    
    return obstacle_list

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

# plt.show()