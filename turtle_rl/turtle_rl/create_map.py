import numpy as np
import matplotlib.pyplot as plt

"""Convert [i][j] in list to (x, y) location.

Keyword arguments:
i -- row index
j -- column index
dx -- dx = $(length_of_the_map) / 2 * (-1)
dy -- dy = $(width_of_the_map) / 2 * (-1)
d -- division of one meter, which represents the resolution of the map
"""
def ij2xy(i, j, dx=-6.0, dy=-6.0, d=100):
    x = j/d + dx
    y = i/d + dy
    return x, y

def xy2ij(x, y, dx=-6.0, dy=-6.0, d=100):
    i = int((y - dy)*d)
    j = int((x - dx)*d)
    return i, j

def normalize_angle(rad):
    if abs(rad) >= 2*np.pi:
        two_pi = int(rad / (2 * np.pi))
        rad -= two_pi * 2 * np.pi
    if rad >= np.pi:
        rad -= 2*np.pi
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
            return 7, 7, 6.28, -1, [], success

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
            return 7, 7, 6.28, -1, [], success

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
            return 7, 7, 6.28, -1, -1, [], success

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
            return 7, 7, 6.28, -1, -1, [], success

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
        return 7, 7, -1, [], success
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
        if obstacle_list[i][j] == 1:
            return True
    for pixel in pending_list:
        [i, j] = pixel
        obstacle_list[i][j] = 1
    return False
        
def create_map(
        map_length = 12,
        map_width = 12,
        divison = 100,
        randomness = 1,
):
    i_max = int(map_width*divison + 1)
    j_max = int(map_length*divison + 1)
    obstacle_list = np.zeros(shape=(i_max, j_max))
    square_list = []
    rectangle_list = []
    cylinder_list = []
    
    # Generate fixed number of obstacles: 2 squares + 4 rectangles + 2 cylinders
    # Obstacles has fixed default orientation: 0.0
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
    # Obstacles has random orientation
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
    
    return obstacle_list, square_list, rectangle_list, cylinder_list

ol, sl, rl, cy = create_map(randomness=2)
xl = []
yl = []
for i in range(1201):
    for j in range(1201):
        if ol[i][j] == 1:
            x, y = ij2xy(i, j)
            xl.append(x)
            yl.append(y)
plt.plot(xl, yl, 'r.')
plt.xlim([-6, 6])
plt.ylim([-6, 6])
plt.show()
