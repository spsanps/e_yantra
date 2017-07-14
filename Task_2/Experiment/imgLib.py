# -*- coding: utf-8 -*-
"""
**************************************************************************
*                  Cross_A_Crater (e-Yantra 2016)
*                  ================================
*  This software is intended to teach image processing concepts
*
*  MODULE: Task2
*  Filename: imgLib.py
*  Version: 1.5.0  
*  Date: November 21, 2016
*  
*  Author: Jayant Solanki, e-Yantra Project, Department of Computer Science
*  and Engineering, Indian Institute of Technology Bombay.
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
*     
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
"""
# Complete the both function mentioned below, and return the desired outputs
# Additionally you may add your own methods here to help both methods mentioned below
###################Do not add any external libraries#######################
from itertools import product

import cv2
import numpy as np


# detectCellVal detects the numbers/operatorsm,
# perform respective expression evaluation
# and stores them into the grid_map 
# detectCellVal(img,grid_map)
# Find the number/operators, perform the calculations and store the result into the grid_map
# Return the resultant grid_map

def read_cell_from_grid(image, x, y, x_cell_size=50, y_cell_size=50, side=24):  # side cuts out the sides
    """
    Generic function used to get a cell from the image of a matrix. Also used in task1.
    """
    y_start = y * y_cell_size + side
    y_end = (y + 1) * y_cell_size - side
    x_start = x * x_cell_size + side
    x_end = (x + 1) * x_cell_size - side
    return image[y_start:y_end, x_start:x_end]


def xy_to_px(x, y):
    img_x_cell = 50
    img_y_cell = 50
    return (x - 1) * img_x_cell + 25, (y - 1) * img_y_cell + 25
    # -1 to convert from e-yantra numbering to sensible numbering convention


def draw_line(point1, point2, img):
    x1, y1 = xy_to_px(point1[0], point1[1])
    x2, y2 = xy_to_px(point2[0], point2[1])
    cv2.line(img, (x1, y1), (x2, y2), (199, 99, 27), 2)


def draw_route(route, img):
    for i in xrange(len(route) - 1):
        draw_line(route[i], route[i + 1], img)


def detectCellVal(img_gray, grid_map):
    """
       Description of function mapper:

       Detection works by checking whether the pixels at the center of each cell is black or white. Ones will
       have black pixels at the center and zeros will have white.

       Although task 1 required template matching, it is not required here as this is an
       easy (and very very fast) work around.
       """
    mapper = lambda x, y: int(np.sum(read_cell_from_grid(img_gray, y, x)) < 500)
    # 500 is found after checking np.sum values. ~ 1000 for zero and ~ 5 for 1. A middle value of 500 is used
    grid_map = np.fromfunction(np.vectorize(mapper), (14, 14), dtype=int)
    # vectorization is 10% faster than nested for (tested for 10**4 loops) although essentially it's the same
    return grid_map.tolist()


############################################################################################
# solveGrid finds the shortest path,
# between valid grid cell in the start row 
# and valid grid cell in the destination row 
# solveGrid(grid_map)
# Return the route_path and route_length

def generate_next(current):
    """
    Possible points to move to from current position. Made for generalising the search algorithm
    """
    x, y = current
    possible_x = xrange(max(x - 1, 0), min(x + 2, 13))  # possible x coordinates
    possible_y = xrange(max(y - 1, 0), min(y + 2, 13))  # possible y coordinates
    return product(possible_x, possible_y)  # possible cells is the cartesian product


def next_step(grid_traversed, front):
    """
    new_front is where search has reached after the step
    :param grid_traversed: Storing search info. Already searched points ..etc
    :param front: Point at where search has reached before this step
    """
    new_front = []
    for point in front:
        for next_point in generate_next(point):
            x, y = next_point
            if grid_traversed[y][x] is 1:
                grid_traversed[y][x] = point  # Storing where the front has come from for trace back
                if y == 0: return []  # If end point is reached stop all fronts. ie return no front
                new_front.append(next_point)
    return new_front


def search_from(front, grid_traversed):
    for point in front:
        x, y = point
        grid_traversed[y][x] = 0  # to avoid looping back to start point
    while len(front) > 0:
        front = next_step(grid_traversed, front)


def trace_route_back(point, grid_traversed):
    """
    Traces the route back from the point given using the grid_traversed map
    """
    x, y = point
    path = [point]
    while y != 13:
        point = grid_traversed[y][x]
        path.append(point)
        x, y = point
    return path[::-1]  # reverse to show from start to end


def solver(grid_map):
    """
    Essentially Dijkstra's algorithm. With "wavefront" starting at all possible start points.
    For such small grids with these many obstacles A* will provided very little speed up or might even slow
    it down especially if the multiple starting points are factored in.
    """
    grid_traversed = [list(row) for row in grid_map]  # Create a copy of the map
    start_points = [(x, 13) for x in xrange(14) if grid_map[13][x] is 1]

    search_from(start_points, grid_traversed)

    for col_no in xrange(14):
        if type(grid_traversed[0][col_no]) is not int:  # it will be a tuple if search has reached this point before
            route = trace_route_back((col_no, 0), grid_traversed)
            return [(x + 1, y + 1) for x, y in route], len(route) - 1  # route, route_len
            # adding 1 to x, y to follow e-yantra numbering convention

    return [], 0


def solveGrid(grid_map):
    route_path, route_length = solver(grid_map)
    return route_path, route_length
    ############################################################################################
