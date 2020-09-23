
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  2 14:03:03 2019

@author: jipengda
"""
# if a request i has a desired departure time Ai, the latest time the vehicle may arrive at the origin node is defined as Bi=
# Bi=Ai+a,with a < 45 minutes depending on the priority of the request
import math
import numpy as np
def create_coord_x(colNumber, rowNumber): 
    nodeNumber=colNumber * rowNumber
    coord_x=[0 for node_0 in range(nodeNumber)]
    for node in range(nodeNumber):
        coord_x[node]=node % colNumber
    return coord_x

def create_coord_y(colNumber, rowNumber): 
    nodeNumber=colNumber * rowNumber
    coord_y=[0 for node_0 in range(nodeNumber)]
    for node in range(nodeNumber):
        coord_y[node]=math.floor(node / colNumber)
    return coord_y
 
def create_D(nodesNumber, coord_x, coord_y):
    n = nodesNumber
    D = [[0 for i_i in range(n)] for j_j in range(n)]
    nodes=range(0, n)
    for i in nodes:
        for j in nodes:
            D[i][j]=np.hypot(coord_x[i]-coord_x[j],coord_y[i]-coord_y[j])                  
    return D
            
    # within manhatan distance, two nodes has eura distance. Or, two nodes has infinity distance.    







    



      
