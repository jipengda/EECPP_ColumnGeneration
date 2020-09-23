# -*- coding: utf-8 -*-
"""
Created on Tue Apr 14 09:57:20 2020

@author: Jipeng
"""
import numpy as np
import math
class Data():
    
    def __init__(self, colNumber, rowNumber):
        self.c = colNumber
        self.r = rowNumber
    def create_coord_x(self):
        nodeNumber = self.c * self.r
        coord_x = [0 for node in range(nodeNumber)]
        for node in range(nodeNumber):
            coord_x[node] = node % self.c
        return coord_x
    def create_coord_y(self):
        nodeNumber = self.c * self.r
        coord_y = [0 for node in range(nodeNumber)]
        for node in range(nodeNumber):
            coord_y[node] = math.floor(node / self.c)
        return coord_y
    
def graphInitialize(graph, colNumber, rowNumber, obstacle, x, y):
    lamda = 0.1164
    nodes = colNumber * rowNumber
    # judge is False, means x node and y node interacts with obstacle
    # judge is True, means x node and y node not internacts with obstacle
    judge = True 
    for u in range(nodes):
        for v in range(nodes):
            judge = Elimination_based_on_obstacle(colNumber, obstacle, u, v)
            if judge == False:
                pass
            elif u in obstacle or v in obstacle:
                pass
            else:
                graph[u][v] = lamda * np.hypot(x[u] - x[v], y[u] - y[v])
    return graph

def Elimination_based_on_obstacle(colNumber, obstacles, lastNode, newNode):
    sideLength = 1
    flag=1 # initialize flag=1
    for obstacle in obstacles:
        x3 = obstacle % colNumber
        y3 = obstacle // colNumber
        r_xmin = x3 - sideLength/2
        r_xmax = x3 + sideLength/2
        r_ymin = y3 - sideLength/2
        r_ymax = y3 + sideLength/2
        if RSIntersection(colNumber,r_xmin, r_xmax, r_ymin, r_ymax, lastNode, newNode) == 1:
            flag = 0 #flag is variable to replace directly return False of True because we need to test all obstacles
        else:
            flag = 1
        if flag == 0:#once flag change to 0, means one obstacle is hit, thus new node can not be added
            return False
    return True# This means all obstacles are not hit, new node can be considered to be added

def IntervalOverlap(x1, x2, x3, x4):
    t = 0    
    if x3 > x4:
       t = x3
       x3 = x4	   
       x4 = t
    if x3 >= x2 or x4 <= x1:
        return 0
    else:
        return 1
    
"""
 * judge rectangular r and line segment AB has intersection or not，if yes: return 1，if no: return 0
"""
def RSIntersection(colNumber,r_xmin, r_xmax, r_ymin, r_ymax, nodeA, nodeB):
    A_x = nodeA % colNumber
    A_y = nodeA // colNumber
    B_x = nodeB % colNumber
    B_y = nodeB // colNumber
    if (A_y == B_y):# if line segment parallel to x axis
        if A_y <= r_ymax and A_y >= r_ymin:
            return IntervalOverlap(r_xmin, r_xmax, A_x,B_x)
        else:
            return 0

	# exchange point A and point B，make point B y coordinate biggest

    # Exchange node A and node B, let B's y value is bigger
    t = 0
    if A_y > B_y:
       t = A_y
       A_y = B_y
       B_y = t
       t= A_x
       A_x = B_x
       B_x=t
	
    # In line segment AB, to find point C and D
    # Two points secure a line: (x-x1)/(x2-x1)=(y-y1)/(y2-y1)
    k = (B_x - A_x)/(B_y - A_y)
    if A_y < r_ymin:
       D_y = r_ymin
       D_x = k*(D_y - A_y) + A_x
    else:
       D_y=A_y
       D_x=A_x
    if B_y > r_ymax:
       C_y = r_ymax
       C_x = k*(C_y-A_y) + A_x
    else:
       C_y = B_y
       C_x = B_x
    if C_y >= D_y: # y axis has overlap
       return IntervalOverlap(r_xmin, r_xmax,D_x, C_x)
    else:
       return 0
#end

# need update because we have different formal paramter and virtual parameter
def angle(last, current, new, vertices):
    colNumber = int(math.sqrt(vertices))
    o=last
    p=current
    q=new
    # need redefine coord_x and coord_y based on vertices
    coord_x = [0 for node in range(vertices)]
    for node in range(vertices):
        coord_x[node] = node % colNumber
    coord_y = [0 for node in range(vertices)]
    for node in range(vertices):
        coord_y[node] = math.floor(node / colNumber) 
    radians_to_degrees = 180/(math.pi)
    theta_radians=0
    theta_degrees=0
    distance_o_p=distance(o,p,coord_x,coord_y)
    distance_p_q=distance(p,q,coord_x,coord_y)
    distance_o_q=distance(o,q,coord_x,coord_y)
    theta_radians=math.pi-np.arccos(round((distance_o_p**2+distance_p_q**2-distance_o_q**2)/(2*distance_o_p*distance_p_q),2))
    theta_degrees=theta_radians * radians_to_degrees
    return theta_degrees
def distance(firstNode, secondNode,coord_x,coord_y):
    i=firstNode
    j=secondNode
    distanceValue=0
    distanceValue=np.hypot(coord_x[i]-coord_x[j],coord_y[i]-coord_y[j])
    return distanceValue


        