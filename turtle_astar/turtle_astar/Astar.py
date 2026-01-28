import numpy as np
from Obstacles import *

LENGTH = 12.0
WIDTH = 12.0
ONE_METER = 100
# TODO Chnage URDF later
collision_radius = 0.11

def chebyshev_distance(m, n):
    row_diff = np.sqrt((n[0] - m[0])**2)
    column_diff = np.sqrt((n[1] - m[1])**2)
    if row_diff >= column_diff:
        return row_diff
    else:
        return column_diff
    
def eulidean_distance(m, n):
    return np.sqrt((n[0] - m[0])**2 + (n[1] - m[1])**2)

def heuristic(curr, goal):
    return chebyshev_distance(curr, goal)

def reconstruct_path(came_from, goal):
    curr = goal
    path = [curr]
    prev = came_from[goal]
    try:
        while True:
            curr = prev
            path.insert(0, curr)
            prev = came_from[curr]
    except KeyError as e:
        return path

def find_neighbors(curr):
    i_max = int(WIDTH*ONE_METER + 1)
    j_max = int(LENGTH*ONE_METER + 1)
    r = curr[0]
    c = curr[1]
    if curr[0] > 0:
        if curr[0] < i_max - 1:
            if curr[1] > 0:
                # body
                if curr[1] < j_max - 1:
                    return [(r+1,c-1), (r+1,c), (r+1,c+1), (r,c-1), (r,c+1), (r-1,c-1), (r-1,c), (r-1,c+1)]
                # right edge
                else:
                    return [(r+1,c-1), (r+1,c), (r,c-1), (r-1,c-1), (r-1,c)]
            # left edge
            else:
                return [(r+1,c), (r+1,c+1), (r,c+1), (r-1,c), (r-1,c+1)]
        else:
            if curr[1] > 0:
                # top edge
                if curr[1] < j_max - 1:
                    return [(r,c-1), (r,c+1), (r-1,c-1), (r-1,c),(r-1,c+1)]
                # top right corner
                else:
                    return [(r,c-1), (r-1,c-1), (r-1,c)]
            # top left corner
            else:
                return[(r,c+1), (r-1,c), (r-1,c+1)]
    else:
        if curr[1] > 0:
            # bottom edge
            if curr[1] < j_max - 1:
                return [(r+1,c-1), (r+1,c), (r+1,c+1), (r,c-1), (r,c+1)]
            # bottom right corner
            else:
                return [(r+1,c-1), (r+1,c), (r,c-1)]
        # bottom left corner
        else:
            return [(r+1,c), (r+1,c+1), (r,c+1)]

def a_star(
        start_pos, 
        goal_pos, 
        online
):
    i_start, j_start = xy2ij(start_pos[0], start_pos[1])
    i_goal, j_goal = xy2ij(goal_pos[0], goal_pos[1])
    start = (i_start, j_start)
    goal = (i_goal, j_goal)



    open = [start]
    came_from = {}
    online_list = []

    i_max = int(WIDTH*ONE_METER + 1)
    j_max = int(LENGTH*ONE_METER + 1)
    g_scores = np.zeros(shape=(i_max, j_max)) + 999999
    g_scores[start[0]][start[1]] = 0
    f_scores = np.zeros(shape=(i_max, j_max)) + 999999
    f_scores[start[0]][start[1]] = heuristic(start, goal)
    while len(open) > 0:
        # Find the node in open set having the lowest f score
        min_node = ()
        min_f = 1000000
        for node in open:
            if f_scores[node[0]][node[1]] < min_f:
                min_node = node
                min_f = f_scores[node[0]][node[1]]
            # There is a tie between two nodes' f scores
            elif f_scores[node[0]][node[1]] == min_f:
                min_node_to_goal = eulidean_distance(min_node, goal)
                node_to_goal = eulidean_distance(node, goal)
                if min_node_to_goal > node_to_goal:
                    min_node = node
                    min_f = f_scores[node[0]][node[1]]
        curr_node = min_node
        online_list.append(curr_node)

        # Goal node has been reached. Return the path.
        if curr_node == goal:
            if online:
                return online_list
            else:
                return reconstruct_path(came_from, goal)
        
        # Remove current node from open set
        open.remove(curr_node)

        # Find neighbors of current node and analyze them
        neighbors = find_neighbors(curr_node)
        for neighbor in neighbors:
            cost = 1
            x, y = ij2xy(neighbor[0], neighbor[1])
            if obstacle_bumped((x, y), collision_radius):
                cost = 1000
            g_score = g_scores[curr_node[0]][curr_node[1]] + cost
            if g_score < g_scores[neighbor[0]][neighbor[1]]:
                # This path is better than any previous one
                came_from[neighbor] = curr_node
                g_scores[neighbor[0]][neighbor[1]] = g_score
                f_scores[neighbor[0]][neighbor[1]] = g_score + heuristic(neighbor, goal)
                if neighbor not in open:
                    open.append(neighbor)
    return []
