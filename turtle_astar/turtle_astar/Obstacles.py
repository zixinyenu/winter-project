import numpy
import matplotlib.pyplot as plt

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
    return rectangle_bumped(
        turtle_pos, 
        turtle_r, 
        northwestern_pos, 
        southeastern_pos
    )
