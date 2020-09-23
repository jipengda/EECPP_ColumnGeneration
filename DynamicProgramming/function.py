#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 11 23:26:27 2019

@author: jipeng
"""
import math
import numpy as np

def totalCost_calculation(distance_lambda, turn_gamma, optimalSet,colNumber, rowNumber, obstacles):
    totalCost = 0    
    totalCost= dijkstra_totalCost_calculation(distance_lambda, turn_gamma, optimalSet, colNumber,rowNumber, obstacles)
    
    return totalCost

#------------------------------------------------------------------------------
# dijkstra part for case where there is obstacle
#------------------------------------------------------------------------------
def dijkstra_totalCost_calculation(distance_lambda, turn_gamma, optimalSet, colNumber, rowNumber, obstacles):
    import sys
    import Common_module_smile
    class Graph():
    
        def __init__(self, vertices, obstacles):
            self.V = vertices
            self.O = obstacles
            self.graph = [ [sys.maxsize for i in range(vertices)] for j in range(vertices) ]
            
        def printSolution(self, dist):
            print("Vertex \tDistance from Source ")
            for node in range(self.V):
                print(node, "\t", dist[node])
            
        def minDistance(self, dist, sptSet):
            min = sys.maxsize
            for v in range(self.V):            
                if dist[v] < min and sptSet[v] == False:
                    min = dist[v]
                    min_index = v
                
            return min_index
        
        # Function that implements Dijkstra's single source
        # shortest path algorithm 
        def dijkstra(self, src):
            turn_gamma = 0.0173
            dist = [sys.maxsize] * self.V
            pre = [src] * self.V
            path = [[] for i in range(self.V)]
            dist[src] = 0
            sptSet = [False] * self.V
            n = len(self.O)
            nodes= [node for node in range(self.V) if node not in self.O]
            for cout in range(self.V-n):          
                # Pick the minimum vertex from
                # the set of vertices not yet visited.
                # u is always equal to the src in first iteration
                # last shold equal to pre[u]
                u = self.minDistance(dist, sptSet)
                path[u].append(u)
                current = u
                last = pre[u]
                # Put minimum distance vertex in the
                # shortest path tree
                sptSet[u] = True
                # update dist value of the adjacent vertices
                # of the picked vertex only if the current distance
                # is greated than new distance and 
                # the vertex not in shortest path tree
                for v in nodes:
                    if last == current:
                        turnCost = 0
                    else:
                        turnCost = turn_gamma * Common_module_smile.angle(last, current, v, self.V)
                    if self.graph[u][v] > 0:
                        if sptSet[v] == False:
                            if dist[v] >  dist[u] + self.graph[u][v] + turnCost:
                                dist[v] =  dist[u] + self.graph[u][v] + turnCost
                                pre[v] = u
                                # path for each vertice needs recording
                                # need comments to make third person understand
                                # at same time, update path[v]
                                path[v] =[]
                                for node in path[u]:
                                    path[v].append(node)
                            else:
                                pass
            #self.printSolution(dist)
            print("Vertex from Source",src, "\t Path")
            for i in range(self.V):
                path[i].append([dist[i]])
                print(i, "\t", path[i])
            return path
        
    # Driver program
    vertices = colNumber * rowNumber
#    obstacles = [4]
    # define distance cost coefficient and turn cost coefficient
    distance_lambda = 0.1164
    turn_gamma = 0.0173
    
    # Define nodes in vertices any node except obstacle
    nodes = [node for node in range(vertices) if node not in obstacles]
    src = 0
    g = Graph(vertices, obstacles)
    d = Common_module_smile.Data(colNumber, rowNumber)
    coord_x = d.create_coord_x()
    coord_y = d.create_coord_y()
    g.graph = Common_module_smile.graphInitialize(g.graph, colNumber, rowNumber, obstacles, coord_x, coord_y)
    minDistance = [ [sys.maxsize for i in range(vertices)] for j in range(1) ]
    
    minPath = [[] for i in range(vertices)]
    matrix_minPath = [ minPath for i in range(vertices)]
    path_minPath = [[] for i in range(vertices)]
    matrix_path_minPath = [ path_minPath for i in range(vertices)]
    cost_minPath = [0 for i in range(vertices)]
    matrix_cost_minPath = [ cost_minPath for i in range(vertices)]
    for source in nodes:
        minPath = g.dijkstra(source)
        matrix_minPath[source] = minPath
        for unit in range(len(minPath)):
            path_minPath[unit] = list (minPath[unit][:-1])
            cost_minPath[unit] = minPath[unit][-1][0]
        matrix_path_minPath[source]=list(path_minPath)
        matrix_cost_minPath[source]=list(cost_minPath)
    
    arc = [(i,j,k) for i in nodes for j in nodes for k in nodes]
    q={(i,j,k):0 for i,j,k in arc}   
    
    for i,j,k in arc:
        if len(matrix_path_minPath[i][j]) > 1:
            real_i = matrix_path_minPath[i][j][-2]
        else:
            real_i = matrix_path_minPath[i][j][-1]
        real_j = matrix_path_minPath[i][j][-1]
        if len(matrix_path_minPath[j][k]) > 1:
            real_k = matrix_path_minPath[j][k][1]
        else:
            real_k = matrix_path_minPath[j][k][0]
        turn_cost = turn_gamma * Common_module_smile.angle(real_i, real_j, real_k, vertices)
        q[(i,j,k)] = turn_cost
        
    edges = [(i,j) for i in nodes for j in nodes]
    c = {(i,j):0 for i,j in edges}
    for i,j in edges:
        c[(i,j)] = matrix_cost_minPath[i][j]   
    # the End
    # cost = c[(0,node)] * 2 + q[(0, node, 0)]
    totalCost = 0
    firstnode = optimalSet[0]
    secondnode = optimalSet[1]
    lastnode = optimalSet[2]
    distanceCost  = c[firstnode,secondnode] * 2
    turnCost = q[firstnode, secondnode, lastnode]
    totalCost = distanceCost + turnCost
    return totalCost
# the End
# cost = c[(0,node)] * 2 + q[(0, node, 0)]