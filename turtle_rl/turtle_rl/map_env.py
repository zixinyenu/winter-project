import numpy as np
import matplotlib.pyplot as plt
import os

"""Convert [i][j] in list to (x, y) location.

Keyword arguments:
i -- row index
j -- column index
dx -- dx = $(length_of_the_map) / 2 * (-1)
dy -- dy = $(width_of_the_map) / 2 * (-1)
d -- division of one meter, which represents the resolution of the map
"""
def ij2xy(i, j, dx=-3.0, dy=-3.0, d=100):
    x = j/d + dx
    y = i/d + dy
    return x, y

def xy2ij(x, y, dx=-3.0, dy=-3.0, d=100):
    i = int((y - dy)*d)
    j = int((x - dx)*d)
    return i, j

def normalize_angle(rad):
    if abs(rad) >= 2*np.pi:
        two_pi = int(rad / (2 * np.pi))
        rad -= two_pi * 2 * np.pi
    if rad > np.pi:
        rad += -2*np.pi
    elif rad < -np.pi:
        rad += 2*np.pi
    return rad

def rot2d(body_x, body_y, rot_x, rot_y, rad):
    # Since the randomly generated rotation angle is initialized as
    # a normalized angle, there is no need to call normalize_angle function
    # rad = normalize_angle(rad)

    # Convert (rot_x, rot_y) from world frame to body frame
    x = rot_x - body_x
    y = rot_y - body_y
    # Perform pure rotation
    xp = x*np.cos(rad) - y*np.sin(rad)
    yp = x*np.sin(rad) + y*np.cos(rad)
    # Convert them back to world frame
    xpp = xp + body_x
    ypp = yp + body_y
    return (xpp, ypp)

def gen_square_atp(map_length, map_width, d, mean_side, noise, rotate=False):
    if not rotate:
        rad = 0.0
    else:
        rad = np.random.uniform(low=-np.pi, high=np.pi)

    x = np.random.uniform(low=-map_length/2, high=map_length/2)
    y = np.random.uniform(low=-map_width/2, high=map_width/2)
    side = np.random.normal(loc=mean_side, scale=noise)
    side = abs(side)

    pending_list = []
    success = True

    ystart = y-side/2
    xstart = x-side/2
    ystop = (y+side/2)+(1/d)
    xstop = (x+side/2)+(1/d)

    if not rotate:
        if ystart < -map_width/2 or xstart < -map_length/2 or \
            ystop > map_width/2 + 1/d or xstop > map_length/2 + 1/d:
            success = False
            return 4, 4, 6.28, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, d)
                pending_list.append([i, j])
    else:
        ne = rot2d(x, y, xstop, ystop, rad)
        nw = rot2d(x, y, xstart, ystop, rad)
        sw = rot2d(x, y, xstart, ystart, rad)
        se = rot2d(x, y, xstop, ystart, rad)

        if abs(ne[0]) > map_length/2 or abs(ne[1]) > map_width/2 or \
            abs(nw[0]) > map_length/2 or abs(nw[1]) > map_width/2 or \
            abs(sw[0]) > map_length/2 or abs(sw[1]) > map_width/2 or \
            abs(se[0]) > map_length/2 or abs(se[1]) > map_width/2:
            success = False
            return 4, 4, 6.28, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                rot_point = rot2d(x, y, xp, yp, rad)
                i, j = xy2ij(rot_point[0], rot_point[1], -map_length/2, -map_width/2, d)
                pending_list.append([i, j])

    return x, y, rad, side, pending_list, success

def gen_rectangle_atp(map_length, map_width, d, mean_len, mean_wdt, noise, rotate=False):
    if not rotate:
        rad = 0.0
    else:
        rad = np.random.uniform(low=-np.pi, high=np.pi)

    x = np.random.uniform(low=-map_length/2, high=map_length/2)
    y = np.random.uniform(low=-map_width/2, high=map_width/2)
    length = np.random.normal(loc=mean_len, scale=noise)
    width = np.random.normal(loc=mean_wdt, scale=noise)
    length = abs(length)
    width = abs(width)

    pending_list = []
    success = True

    ystart = y-width/2
    xstart = x-length/2
    ystop = (y+width/2)+(1/d)
    xstop = (x+length/2)+(1/d)

    if not rotate:
        if ystart < -map_width/2 or xstart < -map_length/2 or \
            ystop > map_width/2 + 1/d or xstop > map_length/2 + 1/d:
            success = False
            return 4, 4, 6.28, -1, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, d)
                pending_list.append([i, j])
    else:
        ne = rot2d(x, y, xstop, ystop, rad)
        nw = rot2d(x, y, xstart, ystop, rad)
        sw = rot2d(x, y, xstart, ystart, rad)
        se = rot2d(x, y, xstop, ystart, rad)

        if abs(ne[0]) > map_length/2 or abs(ne[1]) > map_width/2 or \
            abs(nw[0]) > map_length/2 or abs(nw[1]) > map_width/2 or \
            abs(sw[0]) > map_length/2 or abs(sw[1]) > map_width/2 or \
            abs(se[0]) > map_length/2 or abs(se[1]) > map_width/2:
            success = False
            return 4, 4, 6.28, -1, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                rot_point = rot2d(x, y, xp, yp, rad)
                i, j = xy2ij(rot_point[0], rot_point[1], -map_length/2, -map_width/2, d)
                pending_list.append([i, j])

    return x, y, rad, length, width, pending_list, success

def gen_cylinder_apt(map_length, map_width, d, mean_radius, noise):
    x = np.random.uniform(low=-map_length/2, high=map_length/2)
    y = np.random.uniform(low=-map_width/2, high=map_width/2)
    radius = np.random.normal(loc=mean_radius, scale=noise)
    radius = abs(radius)

    pending_list = []
    success = True

    ystart = y-radius
    xstart = x-radius
    ystop = (y+radius)+(1/d)
    xstop = (x+radius)+(1/d)

    if ystart < -map_width/2 or xstart < -map_length/2 or \
        ystop > map_width/2 + 1/d or xstop > map_length/2 + 1/d:
        success = False
        return 4, 4, -1, [], success
    for yp in np.arange(start=ystart, stop=ystop, step=1/d):
        for xp in np.arange(start=xstart, stop=xstop, step=1/d):
            if (yp - y)**2 + (xp-x)**2 > radius**2:
                continue
            i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, d)
            pending_list.append([i, j])

    return x, y, radius, pending_list, success

def gen_square_atp_p22(
        map_length,
        map_width,
        d,
        square_x,
        square_y,
        square_ori,
        square_side,
        rotate=False):
    if not rotate:
        rad = 0.0
    else:
        rad = square_ori

    x = square_x
    y = square_y
    side = square_side
    side = abs(side)

    pending_list = []
    success = True

    ystart = y-side/2
    xstart = x-side/2
    ystop = (y+side/2)+(1/d)
    xstop = (x+side/2)+(1/d)

    if not rotate:
        if ystart < -map_width/2 or xstart < -map_length/2 or \
            ystop > map_width/2 + 1/d or xstop > map_length/2 + 1/d:
            success = False
            return 4, 4, 6.28, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, d)
                pending_list.append([i, j])
    else:
        ne = rot2d(x, y, xstop, ystop, rad)
        nw = rot2d(x, y, xstart, ystop, rad)
        sw = rot2d(x, y, xstart, ystart, rad)
        se = rot2d(x, y, xstop, ystart, rad)

        if abs(ne[0]) > map_length/2 or abs(ne[1]) > map_width/2 or \
            abs(nw[0]) > map_length/2 or abs(nw[1]) > map_width/2 or \
            abs(sw[0]) > map_length/2 or abs(sw[1]) > map_width/2 or \
            abs(se[0]) > map_length/2 or abs(se[1]) > map_width/2:
            success = False
            return 4, 4, 6.28, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                rot_point = rot2d(x, y, xp, yp, rad)
                i, j = xy2ij(rot_point[0], rot_point[1], -map_length/2, -map_width/2, d)
                pending_list.append([i, j])

    return x, y, rad, side, pending_list, success

def gen_rectangle_atp_p22(
        map_length,
        map_width,
        d,
        rectangle_x,
        rectangle_y,
        rectangle_ori,
        rectangle_len,
        rectangle_wdt,
        rotate=False):
    if not rotate:
        rad = 0.0
    else:
        rad = rectangle_ori

    x = rectangle_x
    y = rectangle_y
    length = rectangle_len
    width = rectangle_wdt
    length = abs(length)
    width = abs(width)

    pending_list = []
    success = True

    ystart = y-width/2
    xstart = x-length/2
    ystop = (y+width/2)+(1/d)
    xstop = (x+length/2)+(1/d)

    if not rotate:
        if ystart < -map_width/2 or xstart < -map_length/2 or \
            ystop > map_width/2 + 1/d or xstop > map_length/2 + 1/d:
            success = False
            return 4, 4, 6.28, -1, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, d)
                pending_list.append([i, j])
    else:
        ne = rot2d(x, y, xstop, ystop, rad)
        nw = rot2d(x, y, xstart, ystop, rad)
        sw = rot2d(x, y, xstart, ystart, rad)
        se = rot2d(x, y, xstop, ystart, rad)

        if abs(ne[0]) > map_length/2 or abs(ne[1]) > map_width/2 or \
            abs(nw[0]) > map_length/2 or abs(nw[1]) > map_width/2 or \
            abs(sw[0]) > map_length/2 or abs(sw[1]) > map_width/2 or \
            abs(se[0]) > map_length/2 or abs(se[1]) > map_width/2:
            success = False
            return 4, 4, 6.28, -1, -1, [], success

        for yp in np.arange(start=ystart, stop=ystop, step=1/d):
            for xp in np.arange(start=xstart, stop=xstop, step=1/d):
                rot_point = rot2d(x, y, xp, yp, rad)
                i, j = xy2ij(rot_point[0], rot_point[1], -map_length/2, -map_width/2, d)
                pending_list.append([i, j])

    return x, y, rad, length, width, pending_list, success

def gen_cylinder_apt_p22(
        map_length,
        map_width,
        d,
        cylinder_x,
        cylinder_y,
        cylinder_radius):
    x = cylinder_x
    y = cylinder_y
    radius = cylinder_radius
    radius = abs(radius)

    pending_list = []
    success = True

    ystart = y-radius
    xstart = x-radius
    ystop = (y+radius)+(1/d)
    xstop = (x+radius)+(1/d)

    if ystart < -map_width/2 or xstart < -map_length/2 or \
        ystop > map_width/2 + 1/d or xstop > map_length/2 + 1/d:
        success = False
        return 4, 4, -1, [], success
    for yp in np.arange(start=ystart, stop=ystop, step=1/d):
        for xp in np.arange(start=xstart, stop=xstop, step=1/d):
            if (yp - y)**2 + (xp-x)**2 > radius**2:
                continue
            i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, d)
            pending_list.append([i, j])

    return x, y, radius, pending_list, success

def check_collision(obstacle_list, pending_list):
    for pixel in pending_list:
        [i, j] = pixel
        if obstacle_list[i][j] == np.uint8(1):
            return True
    for pixel in pending_list:
        [i, j] = pixel
        obstacle_list[i][j] = np.uint8(1)
    return False

########## TURTLE_FUNCTIONS_START ##########
def gen_turtle_apt(map_length, map_width, d, collision_radius):
    x = np.random.uniform(low=-map_length/2 + collision_radius, high=map_length/2 - collision_radius)
    y = np.random.uniform(low=-map_width/2 + collision_radius, high=map_width/2 - collision_radius)
    radius = collision_radius

    pending_list = []

    ystart = y-radius
    xstart = x-radius
    ystop = (y+radius)+(1/d)
    xstop = (x+radius)+(1/d)

    for yp in np.arange(start=ystart, stop=ystop, step=1/d):
        for xp in np.arange(start=xstart, stop=xstop, step=1/d):
            if (yp - y)**2 + (xp-x)**2 > radius**2:
                continue
            i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, d)
            pending_list.append([i, j])

    rad = np.random.uniform(low=-np.pi, high=np.pi)

    return x, y, rad, radius, pending_list, True

def check_turtle_collision(obstacle_list, pending_list):
    for pixel in pending_list:
        [i, j] = pixel
        if obstacle_list[i][j] == 1:
            return True
    return False
########## TURTLE_FUNCTIONS_END ##########

def create_empty_map(
    map_length = 6,
    map_width = 6,
    divison = 100,
    goal_x = 1.5,
    goal_y = 0.0
):
    i_max = int(map_width*divison + 1)
    j_max = int(map_length*divison + 1)
    obstacle_list = np.zeros(shape=(i_max, j_max), dtype=np.uint8)
    turtle_goal = [goal_x, goal_y]
    return obstacle_list, turtle_goal

def create_one_obstacle_map(
    map_length = 6,
    map_width = 6,
    divison = 100,
    goal_x = 1.5,
    goal_y = 0.0
):
    i_max = int(map_width*divison + 1)
    j_max = int(map_length*divison + 1)
    obstacle_list = np.zeros(shape=(i_max, j_max), dtype=np.uint8)
    for i in range(int((j_max-1)/2 - 0.50*divison + 1),
                    int((j_max-1)/2 + 0.50*divison + 1)):
        for j in range(int((i_max-1)/2 + 0.75*divison + 1),
                    int((i_max-1)/2 + 1.00*divison + 1)):
            obstacle_list[i][j] = np.uint8(1)
    turtle_goal = [goal_x, goal_y]
    return obstacle_list, turtle_goal

def create_eight_obstacle_map(
    map_length = 6,
    map_width = 6,
    divison = 100,
    goal_x = 1.5,
    goal_y = 0.0
):
    i_max = int(map_width*divison + 1)
    j_max = int(map_length*divison + 1)
    obstacle_list = np.zeros(shape=(i_max, j_max), dtype=np.uint8)
    square_list = []
    rectangle_list = []
    cylinder_list = []

    # 12 o'clock
    x, y, rad, side, pending_list, success = gen_square_atp_p22(map_length, map_width, divison, 0.75, 0.0, 0.0, 0.3, True)
    check_collision(obstacle_list, pending_list)
    square_list.append([x, y, rad, side])
    # 1.5 o'clock
    x, y, rad, length, width, pending_list, success = gen_rectangle_atp_p22(map_length, map_width, divison, 0.75, -0.75, 0.78, 0.45, 0.05, True)
    check_collision(obstacle_list, pending_list)
    rectangle_list.append([x, y, rad, length, width])
    # 3 o'clock
    x, y, radius, pending_list, success = gen_cylinder_apt_p22(map_length, map_width, divison, 0.0, -0.9, 0.15)
    check_collision(obstacle_list, pending_list)
    cylinder_list.append([x, y, radius])
    # 4.5 o'clock
    x, y, rad, side, pending_list, success = gen_square_atp_p22(map_length, map_width, divison, -0.5, -0.5, 0.78, 0.2, True)
    check_collision(obstacle_list, pending_list)
    square_list.append([x, y, rad, side])
    # 6 o'clock
    x, y, rad, length, width, pending_list, success = gen_rectangle_atp_p22(map_length, map_width, divison, -1.1, 0.0, 1.57, 0.55, 0.05, True)
    check_collision(obstacle_list, pending_list)
    rectangle_list.append([x, y, rad, length, width])
    # 7.5 o'clock
    x, y, radius, pending_list, success = gen_cylinder_apt_p22(map_length, map_width, divison, -0.7, 0.7, 0.1)
    check_collision(obstacle_list, pending_list)
    cylinder_list.append([x, y, radius])
    # 9 o'clock
    x, y, rad, side, pending_list, success = gen_square_atp_p22(map_length, map_width, divison, 0.0, 0.75, 0.0, 0.25, True)
    check_collision(obstacle_list, pending_list)
    square_list.append([x, y, rad, side])
    # 10.5 o'clock
    x, y, rad, length, width, pending_list, success = gen_rectangle_atp_p22(map_length, map_width, divison, 0.80, 0.80, -0.78, 0.65, 0.05, True)
    check_collision(obstacle_list, pending_list)
    rectangle_list.append([x, y, rad, length, width])

    # dir_path = "/home/zixin/ws/winter_project/src/turtle_rl/worlds/P22_obstacles"
    # square_count = 0
    # for square in square_list:
    #     create_squares(square, square_count, dir_path)
    #     square_count += 1
    # rectangle_count = 0
    # for rectangle in rectangle_list:
    #     create_rectangles(rectangle, rectangle_count, dir_path)
    #     rectangle_count += 1
    # cylinder_count = 0
    # for cylinder in cylinder_list:
    #     create_cylinders(cylinder, cylinder_count, dir_path)
    #     cylinder_count += 1
    
    turtle_goal = [goal_x, goal_y]
    return obstacle_list, turtle_goal
        
def create_map(
        map_length = 6,
        map_width = 6,
        divison = 100,
        collision_radius=0.11,
        randomness = 1,
        border = 2.5
):
    i_max = int(map_width*divison + 1)
    j_max = int(map_length*divison + 1)
    obstacle_list = np.zeros(shape=(i_max, j_max), dtype=np.uint8)
    square_list = []
    rectangle_list = []
    cylinder_list = []
    turtle_param = []
    turtle_goal = [-4.0, -4.0]
    
    # Generate fixed number of obstacles: 2 squares + 4 rectangles + 2 cylinders
    # Obstacles have random positions and a fixed default orientation: 0.0
    # Obstacles of same type tend to have similar size
    if randomness == 1:
        square_count = 0
        while square_count < 2:
            x, y, rad, side, pending_list, success = gen_square_atp(map_length, map_width, divison, 1.0, 0.1)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            square_list.append([x, y, rad, side])
            square_count += 1

        rectangle_count = 0
        while rectangle_count < 4:
            x, y, rad, length, width, pending_list, success = gen_rectangle_atp(map_length, map_width, divison, 2.7, 1.3, 0.2)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            rectangle_list.append([x, y, rad, length, width])
            rectangle_count += 1

        cylinder_count = 0
        while cylinder_count < 2:
            x, y, radius, pending_list, success = gen_cylinder_apt(map_length, map_width, divison, 0.75, 0.05)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            cylinder_list.append([x, y, radius])
            cylinder_count += 1
    # Generate fixed number of obstacles: 2 squares + 4 rectangles + 2 cylinders
    # Obstacles have random positions and orientations
    # Obstacles of same type tend to have relatively different size
    elif randomness == 2:
        square_count = 0
        while square_count < 2:
            x, y, rad, side, pending_list, success = gen_square_atp(map_length, map_width, divison, 1.0, 0.3, True)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            square_list.append([x, y, rad, side])
            square_count += 1

        rectangle_count = 0
        while rectangle_count < 4:
            x, y, rad, length, width, pending_list, success = gen_rectangle_atp(map_length, map_width, divison, 2.7, 1.3, 0.4, True)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            rectangle_list.append([x, y, rad, length, width])
            rectangle_count += 1

        cylinder_count = 0
        while cylinder_count < 2:
            x, y, radius, pending_list, success = gen_cylinder_apt(map_length, map_width, divison, 0.75, 0.15)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            cylinder_list.append([x, y, radius])
            cylinder_count += 1

    # Generate random number of each type of obstacles: squares + rectangles + cylinders
    # Obstacles have random positions and orientations
    # Obstacles of same type tend to have relatively more different size
    # More realistic for the problem framing of the turtle_rl package
    elif randomness == 3:
        total = 18

        square_total = np.clip(int(np.random.normal(loc=3, scale=1)), a_min=1, a_max=None)
        rectangle_total = np.clip(int(np.random.normal(loc=10, scale=2)), a_min=1, a_max=None)
        cylinder_total = np.clip(total - square_total - rectangle_total, a_min=1, a_max=None)

        small_square_total = np.random.randint(low=1, high=square_total+1)
        big_square_total = square_total - small_square_total
        slim_rectangle_total = np.random.randint(low=1, high=rectangle_total+1)
        thick_rectangle_total = rectangle_total - slim_rectangle_total
        small_cylinder_total = np.random.randint(low=1, high=cylinder_total+1)
        big_cylinder_total = cylinder_total - small_cylinder_total

        small_square_count = 0
        while small_square_count < small_square_total:
            x, y, rad, side, pending_list, success = gen_square_atp(map_length, map_width, divison, 0.3, 0.1, True)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            square_list.append([x, y, rad, side])
            small_square_count += 1

        big_square_count = 0
        while big_square_count < big_square_total:
            x, y, rad, side, pending_list, success = gen_square_atp(map_length, map_width, divison, 0.6, 0.15, True)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            square_list.append([x, y, rad, side])
            big_square_count += 1

        slim_rectangle_count = 0
        while slim_rectangle_count < slim_rectangle_total:
            x, y, rad, length, width, pending_list, success = gen_rectangle_atp(map_length, map_width, divison, 0.8, 0.15, 0.05, True)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            rectangle_list.append([x, y, rad, length, width])
            slim_rectangle_count += 1

        thick_rectangle_count = 0
        while thick_rectangle_count < thick_rectangle_total:
            x, y, rad, length, width, pending_list, success = gen_rectangle_atp(map_length, map_width, divison, 0.5, 0.3, 0.05, True)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            rectangle_list.append([x, y, rad, length, width])
            thick_rectangle_count += 1

        small_cylinder_count = 0
        while small_cylinder_count < small_cylinder_total:
            x, y, radius, pending_list, success = gen_cylinder_apt(map_length, map_width, divison, 0.2, 0.05)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            cylinder_list.append([x, y, radius])
            small_cylinder_count += 1

        big_cylinder_count = 0
        while big_cylinder_count < big_cylinder_total:
            x, y, radius, pending_list, success = gen_cylinder_apt(map_length, map_width, divison, 0.4, 0.1)
            if not success:
                continue
            flag = check_collision(obstacle_list, pending_list)
            if flag:
                continue
            cylinder_list.append([x, y, radius])
            big_cylinder_count += 1

    turtle_count = 0
    while turtle_count < 2:
        x, y, rad, radius, pending_list, success = gen_turtle_apt(map_length, map_width, divison, collision_radius)
        if not success:
            continue
        flag = check_turtle_collision(obstacle_list, pending_list)
        if flag:
            continue

        if turtle_count == 0:
            turtle_param = [x, y, rad]
            turtle_count += 1
        else:
            start_x = turtle_param[0]
            start_y = turtle_param[1]
            if ((x - start_x)**2 + (y - start_y)**2)**0.5 < border:
                continue
            else:
                turtle_goal[0] = x
                turtle_goal[1] = y
                turtle_count += 1

    return obstacle_list, square_list, rectangle_list, cylinder_list, turtle_param, turtle_goal

def create_new_world(parent_directory_name, world_id):
    child_directory_name  = f'world_episode_{world_id}'
    try:
        os.mkdir(f'{parent_directory_name}/{child_directory_name}')
        print(f"Directory {child_directory_name} created successfully. Path returned.")
        return f'{parent_directory_name}/{child_directory_name}'
    except FileExistsError:
        print(f"Directory {child_directory_name} already exists. Path returned.")
        return f'{parent_directory_name}/{child_directory_name}'
    except PermissionError:
        print(f"Permission denied: unable to create '{child_directory_name}'.")
    except Exception as e:
        print(f"An error occurred: {e}.")

def create_squares(square_param, square_id, world_dir):
    mass = 120
    # square: [x, y, rad, side] - pose does not work, need to specify it in srv
    with open(f"{world_dir}/square_{square_id}.sdf", "x") as f:
        x = square_param[0]
        y = square_param[1]
        oz = square_param[2]
        side = square_param[3]
        ixx = mass / 12 * (side**2 + 1.0**2)
        iyy = mass / 12 * (side**2 + 1.0**2)
        izz = mass / 12 * (2 * side**2)
        f.writelines([
            "<sdf version='1.12'>\n",
            f"  <model name='square_{square_id}'>\n",
            "    <plugin filename=\"gz-sim-touchplugin-system\"\n",
            "            name=\"gz::sim::systems::TouchPlugin\">\n",
            "        <target>turtlebot3_burger</target>\n",
            "        <namespace>obstacle</namespace>\n",
            "        <time>0.001</time>\n",
            "        <enabled>true</enabled>\n",
            "    </plugin>\n",
            f"    <pose>{x} {y} 0.5 0.0 0.0 {oz}</pose>\n",
            "    <link name='box_link'>\n",
            "      <sensor name='sensor_contact' type='contact'>\n",
            "          <contact>\n",
            "              <collision>box_collision</collision>\n",
            "          </contact>\n",
            "      </sensor>\n",
            "      <inertial>\n",
            "        <inertia>\n",
            f"          <ixx>{ixx}</ixx>\n",
            "          <ixy>0</ixy>\n",
            "          <ixz>0</ixz>\n",
            f"          <iyy>{iyy}</iyy>\n",
            "          <iyz>0</iyz>\n",
            f"          <izz>{izz}</izz>\n",
            "        </inertia>\n",
            f"        <mass>{mass}</mass>\n",
            "        <pose>0 0 0 0 0 0</pose>\n",
            "      </inertial>\n",
            "      <collision name='box_collision'>\n",
            "        <geometry>\n",
            "          <box>\n",
            f"            <size>{side} {side} 1.0</size>\n",
            "          </box>\n",
            "        </geometry>\n",
            "        <surface>\n",
            "          <friction>\n",
            "            <ode/>\n",
            "          </friction>\n",
            "          <bounce/>\n",
            "          <contact/>\n",
            "        </surface>\n",
            "      </collision>\n",
            "      <visual name='box_visual'>\n",
            "        <geometry>\n",
            "          <box>\n",
            f"            <size>{side} {side} 1.0</size>\n",
            "          </box>\n",
            "        </geometry>\n",
            "        <material>\n",
            "          <ambient>0 0 1 1</ambient>\n",
            "          <diffuse>0 0 1 1</diffuse>\n",
            "          <specular>0 0 1 1</specular>\n",
            "        </material>\n",
            "      </visual>\n",
            "      <pose>0 0 0 0 0 0</pose>\n",
            "      <enable_wind>false</enable_wind>\n",
            "    </link>\n",
            "    <static>false</static>\n",
            "    <self_collide>false</self_collide>\n",
            "  </model>\n",
            "</sdf>\n",
        ])
    return f"{world_dir}/square_{square_id}.sdf"

def create_rectangles(rectangle_param, rectangle_id, world_dir):
    mass = 120
    # rectangle: [x, y, rad, length, width]
    with open(f"{world_dir}/rectangle_{rectangle_id}.sdf", "x") as f:
        x = rectangle_param[0]
        y = rectangle_param[1]
        oz = rectangle_param[2]
        length = rectangle_param[3]
        width = rectangle_param[4]
        ixx = mass / 12 * (width**2 + 1.0**2)
        iyy = mass / 12 * (length**2 + 1.0**2)
        izz = mass / 12 * (length**2 + width**2)
        f.writelines([
            "<sdf version='1.12'>\n",
            f"  <model name='rectangle_{rectangle_id}'>\n",
            "    <plugin filename=\"gz-sim-touchplugin-system\"\n",
            "            name=\"gz::sim::systems::TouchPlugin\">\n",
            "        <target>turtlebot3_burger</target>\n",
            "        <namespace>obstacle</namespace>\n",
            "        <time>0.001</time>\n",
            "        <enabled>true</enabled>\n",
            "    </plugin>\n",
            f"    <pose>{x} {y} 0.5 0.0 0.0 {oz}</pose>\n",
            "    <link name='box_link'>\n",
            "      <sensor name='sensor_contact' type='contact'>\n",
            "          <contact>\n",
            "              <collision>box_collision</collision>\n",
            "          </contact>\n",
            "      </sensor>\n",
            "      <inertial>\n",
            "        <inertia>\n",
            f"          <ixx>{ixx}</ixx>\n",
            "          <ixy>0</ixy>\n",
            "          <ixz>0</ixz>\n",
            f"          <iyy>{iyy}</iyy>\n",
            "          <iyz>0</iyz>\n",
            f"          <izz>{izz}</izz>\n",
            "        </inertia>\n",
            f"        <mass>{mass}</mass>\n",
            "        <pose>0 0 0 0 0 0</pose>\n",
            "      </inertial>\n",
            "      <collision name='box_collision'>\n",
            "        <geometry>\n",
            "          <box>\n",
            f"            <size>{length} {width} 1.0</size>\n",
            "          </box>\n",
            "        </geometry>\n",
            "        <surface>\n",
            "          <friction>\n",
            "            <ode/>\n",
            "          </friction>\n",
            "          <bounce/>\n",
            "          <contact/>\n",
            "        </surface>\n",
            "      </collision>\n",
            "      <visual name='box_visual'>\n",
            "        <geometry>\n",
            "          <box>\n",
            f"            <size>{length} {width} 1.0</size>\n",
            "          </box>\n",
            "        </geometry>\n",
            "        <material>\n",
            "          <ambient>0 0 1 1</ambient>\n",
            "          <diffuse>0 0 1 1</diffuse>\n",
            "          <specular>0 0 1 1</specular>\n",
            "        </material>\n",
            "      </visual>\n",
            "      <pose>0 0 0 0 0 0</pose>\n",
            "      <enable_wind>false</enable_wind>\n",
            "    </link>\n",
            "    <static>false</static>\n",
            "    <self_collide>false</self_collide>\n",
            "  </model>\n",
            "</sdf>\n",
        ])
    return f"{world_dir}/rectangle_{rectangle_id}.sdf"

def create_cylinders(cylinder_param, cylinder_id, world_dir):
    mass = 120
    # cylinder: [x, y, radius]
    with open(f"{world_dir}/cylinder_{cylinder_id}.sdf", "x") as f:
        x = cylinder_param[0]
        y = cylinder_param[1]
        r = cylinder_param[2]
        ixx = mass / 12 * (3 * r**2 + 1.0**2)
        iyy = mass / 12 * (3 * r**2 + 1.0**2)
        izz = mass / 2 * (r**2)
        f.writelines([
            "<sdf version='1.12'>\n",
            f"  <model name='cylinder_{cylinder_id}'>\n",
            "    <plugin filename=\"gz-sim-touchplugin-system\"\n",
            "            name=\"gz::sim::systems::TouchPlugin\">\n",
            "        <target>turtlebot3_burger</target>\n",
            "        <namespace>obstacle</namespace>\n",
            "        <time>0.001</time>\n",
            "        <enabled>true</enabled>\n",
            "    </plugin>\n",
            f"    <pose>{x} {y} 0.5 0.0 0.0 0.0</pose>\n",
            "    <link name='cylinder_link'>\n",
            "      <sensor name='sensor_contact' type='contact'>\n",
            "          <contact>\n",
            "              <collision>cylinder_collision</collision>\n",
            "          </contact>\n",
            "      </sensor>\n",
            "      <inertial>\n",
            "        <inertia>\n",
            f"          <ixx>{ixx}</ixx>\n",
            "          <ixy>0</ixy>\n",
            "          <ixz>0</ixz>\n",
            f"          <iyy>{iyy}</iyy>\n",
            "          <iyz>0</iyz>\n",
            f"          <izz>{izz}</izz>\n",
            "        </inertia>\n",
            f"        <mass>{mass}</mass>\n",
            "        <pose>0 0 0 0 0 0</pose>\n",
            "      </inertial>\n",
            "      <collision name='cylinder_collision'>\n",
            "        <geometry>\n",
            "          <cylinder>\n",
            f"            <radius>{r}</radius>\n",
            "            <length>1.0</length>\n",
            "          </cylinder>\n",
            "        </geometry>\n",
            "        <surface>\n",
            "          <friction>\n",
            "            <ode/>\n",
            "          </friction>\n",
            "          <bounce/>\n",
            "          <contact/>\n",
            "        </surface>\n",
            "      </collision>\n",
            "      <visual name='cylinder_visual'>\n",
            "        <geometry>\n",
            "          <cylinder>\n",
            f"            <radius>{r}</radius>\n",
            "            <length>1.0</length>\n",
            "          </cylinder>\n",
            "        </geometry>\n",
            "        <material>\n",
            "          <ambient>0 0 1 1</ambient>\n",
            "          <diffuse>0 0 1 1</diffuse>\n",
            "          <specular>0 0 1 1</specular>\n",
            "        </material>\n",
            "      </visual>\n",
            "      <pose>0 0 0 0 0 0</pose>\n",
            "      <enable_wind>false</enable_wind>\n",
            "    </link>\n",
            "    <static>false</static>\n",
            "    <self_collide>false</self_collide>\n",
            "  </model>\n",
            "</sdf>\n",
        ])
    return f"{world_dir}/cylinder_{cylinder_id}.sdf"

# package_path = os.path.abspath(os.path.join(
#     os.getcwd(),
#     os.pardir
# ))
# worlds_path = f'{package_path}/worlds'
# world_path = create_new_world(worlds_path, world_id=0)
# create_squares([[1.0, 1.0, 1.57, 0.5]], 0, world_path)

# ol, sl, rl, cl, tp, tg= create_map(randomness=3)
# xl = []
# yl = []
# for i in range(601):
#     for j in range(601):
#         if ol[i][j] == 1:
#             x, y = ij2xy(i, j)
#             xl.append(x)
#             yl.append(y)
# fig, ax = plt.subplots()
# plt.plot(xl, yl, 'r.', label="obstacles")
# turtle_circle = plt.Circle((tp[0], tp[1]), 0.11, color='blue', label="turtle start position")
# ax.add_patch(turtle_circle)
# plt.arrow(tp[0], tp[1], np.sin(tp[2]), np.cos(tp[2]), width=0.05, label="turtle start orientation")
# plt.plot(tg[0], tg[1], 'g^', label="turtle goal")
# plt.xlim([-3, 3])
# plt.ylim([-3, 3])
# plt.legend(loc="upper right")
# plt.show()

# create_eight_obstacle_map()
