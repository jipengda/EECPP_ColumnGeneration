# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 19:47:25 2020

@author: jipengda
"""

"""
It is expected to be size 10 x 10, or whose grids is between 80 and 140 grids
"""
# referred from largeMap_use data.py

from docplex.mp.model import Model
import matplotlib
import incomplete_DP 
import Data
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import function
import random

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42


#--------------------------------Supplementary functions-----------------------
def check_obstacle(obstacles, lastNode, newNode, colNumber, rowNumber):
    sideLength = 1
    flag=1 # initialize flag=1
    for obstacle in obstacles:
        x3 = obstacle % colNumber
        y3 = obstacle // colNumber
        r_xmin = x3 - sideLength/2
        r_xmax = x3 + sideLength/2
        r_ymin = y3 - sideLength/2
        r_ymax = y3 + sideLength/2
        if RSIntersection(r_xmin, r_xmax, r_ymin, r_ymax, lastNode, newNode, colNumber, rowNumber) == 1:
            flag = 0 #flag is variable to replace directly return False of True because we need to test all obstacles
        else:
            flag = 1
        if flag == 0:#once flag change to 0, means one obstacle is hit, thus new node can not be added
            return False
    return True# This means all obstacles are not hit, new node can be considered to be added
"""
 * if [x1, x2] and [x3, x4] (x4 maybe smaller than x3) has interrection or has not，if yes: return 1， if no: return 0
"""


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
def RSIntersection(r_xmin,r_xmax,r_ymin, r_ymax, nodeA, nodeB, colNumber, rowNumber):
    A_x = nodeA % colNumber
    A_y = nodeA // colNumber
    B_x = nodeB % colNumber
    B_y = nodeB // colNumber
    if (A_y == B_y):# line segement is parallel to x axis//线段平行于x轴
        if A_y <= r_ymax and A_y >= r_ymin:
            return IntervalOverlap(r_xmin, r_xmax, A_x,B_x)
        else:
            return 0

	# Echange point A and point B to let point B has bigger y coordinate//AB两点交换，让B点的y坐标最大

    # Exchange node A and node B, let B's y value is bigger
    t = 0
    if A_y > B_y:
       t = A_y
       A_y = B_y
       B_y = t
       t= A_x
       A_x = B_x
       B_x=t
	
    # In line segment//xianduan AB, to find point C and D
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


#--------------------------------Data-----------------------------------------
agentNumber = 10
colNumber=3
rowNumber=3
coord_x = Data.create_coord_x(colNumber, rowNumber)
coord_y = Data.create_coord_y(colNumber, rowNumber)
Battery_capacity_constraint = 6.0 # 6.0 is test
nodesNumber=rowNumber * colNumber
sideLength=1
departurePoint=0
obstacles=[] # no obstacle
Nodes = [i for i in range(nodesNumber) if i not in obstacles and i!=departurePoint]
NodesAndDeparturePoint=[i for i in range(nodesNumber)]




#-------------------------------get q and sitance parameter--------------------
D=Data.create_D(nodesNumber, coord_x, coord_y)
radians_to_degrees=180/(math.pi)
Nodes=[i for i in range(nodesNumber) if i not in obstacles and i!= departurePoint]
NodesAndDeparturePoint = Nodes + [departurePoint]
edges=[(i,j) for i in NodesAndDeparturePoint for j in NodesAndDeparturePoint]
arcs=[(i,j,k) for i in NodesAndDeparturePoint for j in NodesAndDeparturePoint for k in NodesAndDeparturePoint]
distance_lambda = 0.1164
c={(i,j):0 for i,j in edges}
q={(i,j,k):0 for i,j,k in arcs}
distance={(i,j):0 for i,j in edges}
for i,j in edges:
    distanceValue=np.hypot(coord_x[i]-coord_x[j],coord_y[i]-coord_y[j]) # it is wrong, it does not consider the obstacle between nodes.
    distance[(i,j)]=distanceValue
    distance_cost = distance_lambda * distanceValue
    c[(i,j)] = distance_cost
    

for o,p in edges:
    View = check_obstacle(obstacles, o, p, colNumber, rowNumber)
    if View == 0:
        c[(o,p)] = math.inf
    else:
        pass
    
seq=[-10,-9,-8,-7,-6,-5,-4,-3-2,-1,0,1,2,3,4,5,6,7,8,9,10]
fixed_turn_gamma=0.0173
turn_factor=0.0001 
random.seed(10)   
for i,j,k in arcs:
    turn_gamma = fixed_turn_gamma + random.choice(seq) * turn_factor
    theta_radians=math.pi-np.arccos(round((distance[i,j]**2+distance[j,k]**2-distance[i,k]**2)/(2*distance[i,j]*distance[j,k]),2))
    theta_degrees=theta_radians*radians_to_degrees
    turning_cost=turn_gamma*theta_degrees
    q[(i,j,k)]=turning_cost
    a=math.isnan(turning_cost)
    if a is True:
        turning_cost=0
    else:
        pass
    q[(i,j,k)]=turning_cost


#------------------------------------------------------------------------------
#add automatic_basic_pool part
#------------------------------------------------------------------------------
"""
This file is going to generate basic pool automatically for the map defined

The format of pool is like [0, node, 0, [cost]]
"""
# turning cost is 0, from totalCost_calculation_by_set file we know.
# we may need delete or add somthing to make function.totalCost_calculation can deal
# with case where there is obstacle
# use dijksta for obstalce may not be good enough
# but it can be a choice

# I mean for basic_pool
# we also need use varied turn_gamma
depot = departurePoint
basic_pool=[]
# Nodes will replace (1,nodesNumber)
for node in Nodes:
#for node in range(1, nodesNumber):
    unit_basic_pool=[]
    unit_basic_pool.append(depot)
    unit_basic_pool.append(node)
    unit_basic_pool.append(depot)
    cost = function.totalCost_calculation(distance_lambda,turn_gamma,unit_basic_pool, colNumber, rowNumber, obstacles)
    unit_basic_pool.append([cost])
    basic_pool.append(unit_basic_pool)
        
label_table = basic_pool


def make_eecpp_master_model(label_table, colNumber, rowNumber, **kwargs):
    label_number = len(label_table)
    nodesNumber= colNumber * rowNumber
    departurePoint = 0
    obstacles = []
    ##
    C=[0] * label_number
    for l in range(label_number):
        C[l] = label_table[l][-1][0]

    # column vector set
    a = [[0 for i in range(nodesNumber)] for j in range(label_number)]
    labels = [l for l in range(label_number)]
    for l in range(label_number):
        for j in range(0, nodesNumber):
            if j in label_table[l]:
                a[l][j] = 1
    #model<->m
    m = Model(name='EECPP_master')
    m.labels = labels
    m.label_table = label_table
    m.label_number = label_number
    m.nodes = [i for i in range(nodesNumber)]
    m.x = m.continuous_var_dict(m.labels,lb=0, ub=1, name="visit") # determine if the label is selected(x=1) or not(x=0)
    # minimize total cost
    m.visiting_cost = m.sum( (C[l] * m.x[l]) for l in labels )
    m.minimize(m.visiting_cost)
    

    m.node_visit_cts=[]
    #
    ##########
    for node in m.nodes:
        if node in obstacles:
            node_visit_ct = m.sum(m.x[l] * a[l][node] for l in labels) >= 0
        else:
            node_visit_ct = m.sum(m.x[l] * a[l][node] for l in labels) >= 1
        node_visit_ct_name = 'ct_visit{0!s}'.format(node)
        m.node_visit_cts.append(node_visit_ct)
    m.add_constraints(m.node_visit_cts)
    return m   

def add_pattern_to_master_model(master_model, colNumber, rowNumber, one_candidate, outF):
    
    nodesNumber = colNumber * rowNumber
    departurePoint = 0
    obstacles = []
    Nodes = [i for i in range(nodesNumber) if i not in obstacles and i!=departurePoint]
    NodesAndDeparturePoint = Nodes + [departurePoint]
    gen_model_label = one_candidate[:-2]
    label_cost = one_candidate[-2][0]
    print("Column Generation Label and its cost:")
    LabelAndItsCost="Column Generation Label and its cost:" +"\n" + str(gen_model_label) + "\n" + str(label_cost)
    outF.write(LabelAndItsCost)
    outF.write("\n")
#    print(gen_model_label)

    #
    # last difficult about nameing new_lable and giving the right ID
    new_label=gen_model_label
    new_labelCost = label_cost
    print(new_labelCost)
    new_label.append([new_labelCost])
    max_label_table_id = len(master_model.label_table)
    master_model.label_table.append(new_label)
    new_label_id = max_label_table_id
    master_model.labels.append(new_label_id)
    new_label_x = master_model.continuous_var(lb=0, ub=1, name = 'visit_{0}'.format(new_label_id))
    master_model.x[new_label_id] = new_label_x
    
    
    # update constraints
    for node, ct in zip(master_model.nodes, master_model.node_visit_cts):
        ctlhs = ct.lhs
        if node in new_label:
            ctlhs = ctlhs + new_label_x * 1
            ct.lhs = ctlhs
            
    cost_expr = master_model.visiting_cost
    cost_expr = cost_expr + new_label_x * new_labelCost
    master_model.visiting_cost = cost_expr
    master_model.minimize(master_model.visiting_cost)
    return master_model

def eecpp_solve(q,distance,colNumber, rowNumber, label_table, coord_x, coord_y, **kwargs):
    master_model = make_eecpp_master_model(label_table, colNumber, rowNumber, **kwargs)
    
    tic = time.time()
    
    obj_eps = 1e-4
    rc_eps = 1e-6
    loop_count = 0
    best=0
    curr=1e+20
    SplitLines="*************************************************************"
    ms = None
    # write column generation results to Loop count.txt
    outF = open("Loop count print out.txt", "w")
    # necessay data
    nodesNumber = rowNumber * colNumber
    D=Data.create_D(nodesNumber, coord_x, coord_y)
    Battery_capacity_constraint = 6
    departurePoint = 0
    obstacles = []
    
    
#    while loop_count < 100 and abs(best - curr) >= obj_eps:
    while loop_count < 100:
#    while True:
        ms = master_model.solve(log_output=True)
        loop_count += 1
        best = curr
        curr = master_model.objective_value
        duals = master_model.dual_values(master_model.node_visit_cts)
        # there is still more to be done~~
        one_candidate=incomplete_DP.obtain_one_candidate(q,distance,D, duals, coord_x, coord_y, nodesNumber, Battery_capacity_constraint, departurePoint, obstacles) #add obstacle(m,n)
        compare = []
        if one_candidate == compare:
            print("Fails, one_candidate doesn't exist")
            break
        else:
            pass
        print("*************************************************************")
        print("Column Generation Iteration: ", loop_count)
        CGI_count="Column Generation Iteration: " + str(loop_count)
        print("one_candidate is (nodes, labelCost, reducedCost)")
        print(one_candidate)
        outF.write(SplitLines)
        outF.write("\n")
        outF.write(CGI_count)
        outF.write("\n")
        MM="Master Model Solution and its cost: "
        outF.write(MM)
        outF.write("\n")
        for i in master_model.labels:
            if (master_model.x[i]).solution_value > 0.9:
                ms_label ="master model solution labels: " +  str(master_model.label_table[i])
                outF.write(ms_label)
                outF.write("\n")
        ms_objective = "master model objective: " + str(master_model.objective_value)
        outF.write(ms_objective)
        outF.write("\n")

        Dual_values="Dual values:"+str(duals)
        outF.write(Dual_values)
        outF.write("\n")
        rc_cost = one_candidate[-1]
        rc_cost="rc_cost:"+str(rc_cost)
        outF.write(rc_cost)
        outF.write("\n")
        master_model=add_pattern_to_master_model(master_model, colNumber, rowNumber, one_candidate, outF)
    toc=time.time()
    print("Time taken for solving is " + str((toc-tic)) + "sec")
    print()
    TimeTaken = "Time taken for solving is "+str((toc-tic)) +"sec"
    outF.write(TimeTaken)
    outF.write("\n")
    
#    eecpp_print_solution(master_model, outF)
    outF.close()

def eecpp_solve_default(**kwargs):
    return eecpp_solve(q,distance,colNumber, rowNumber, label_table,coord_x, coord_y, **kwargs)

if __name__ == '__main__':
    s = eecpp_solve_default()
    
#from docplex.mp.model import Model
#import matplotlib
#matplotlib.rcParams['pdf.fonttype'] = 42
#matplotlib.rcParams['ps.fonttype'] = 42
#import incomplete_DP 
#import matplotlib.pyplot as plt
#import matplotlib.patches as patches
#import Data
#import time
#
#from docplex.mp.model import Model
#import Data
#import numpy as np
#import math
#import matplotlib.pyplot as plt
#import time
#
#
## initialize necessary data
#agentNumber = 10
#colNumber=4
#rowNumber=4
#colNumber=4
#rowNumber=5
#Battery_capacity_constraint = 6.0
#nodesNumber=rowNumber * colNumber
#sideLength=1
#departurePoint=0
#obstacles=[]
## initial feasible label pool
##4x4
#all_qualified_labels=[[0, 1, 0, [3.3468]],
#             [0, 2, 0, [3.5796]],
#             [0, 3, 0, [3.8124]],
#             [0, 4, 0, [3.3468]],
#             [0, 5, 0, [3.4432]],
#             [0, 6, 0, [3.6346]],
#             [0, 7, 0, [3.8502]],
#             [0, 8, 0, [3.5796]],
#             [0, 9, 0, [3.6346]],
#             [0, 10, 0, [3.7725]],
#             [0, 11, 0, [3.9534]],
#             [0, 12, 0, [3.8124]],
#             [0, 13, 0, [3.8502]],
#             [0, 14, 0, [3.9534]],
#             [0, 15, 0, [4.1017]]]
#Nodes = [i for i in range(nodesNumber) if i not in obstacles and i!=departurePoint]
##obstacles=[9,10,14,
##           18,21,22,
##           29,30,
##           33,34,38,
##           41,42,
##           49,50,
##           56,57,58,
##           64,65,66]
##obstacles=[30,34,35,40,44,45,54,55,64,65,72,73,74,75,82,83,84]
#NodesAndDeparturePoint=[i for i in range(nodesNumber)]
#coord_x=[]
#coord_y=[]
#coord_x=Data.create_coord_x(colNumber,rowNumber)
#coord_y=Data.create_coord_y(colNumber,rowNumber)
#
#label_table= [[0, 9, 18, 14, 10, 6, 0, [4.869076376472115]],
#	      [0, 13, 17, 16, 12, 8, 4, 0, [4.495260411104263]],
#	      [0, 5, 19, 15, 11, 7, 3, 2, 1, 0, [5.682184093091879]]]
#D=Data.create_D(nodesNumber, coord_x, coord_y)
## change
##Nodes = [i for i in range(nodesNumber) if i != departurePoint and i not in obstacles ]
## end
#tic = time.time()
## one_candidate instead of all_qualified_labels
#one_candidate=incomplete_DP.obtain_one_candidate(D, coord_x, coord_y, nodesNumber, Battery_capacity_constraint, departurePoint, obstacles) #add obstacle(m,n)
#all_qualifed_labels.append(one_candidate)
#label_number = len(all_qualified_labels)
#C=[0] * label_number
#for l in range(label_number):
#    C[l] = all_qualified_labels[l][-1][0]
#a = [[0 for i in range(nodesNumber - 1)] for j in range(label_number)]
#labels = [l for l in range(label_number)]
#
#for i in range(label_number):
#    for j in range(1, nodesNumber):
#        if j in all_qualified_labels[i]:
#            a[i][j-1]=1
#
## An arc flow model for the basic EECPP
#model = Model("EECPP PROBLEM")
#x = model.binary_var_list(labels, name = 'X') # determine if the label is selected(x=1) or not(x=0)
#
#Z = model.sum( (C[l] * x[l]) for l in labels )
#model.minimize(Z)
#
#model.add_constraint(model.sum( x[l] for l in labels )<= agentNumber, ctname = 'agent number limit')
#
#for i in Nodes:
#    model.add_constraint(model.sum( a[l][i-1] * x[l] for l in labels) == 1, ctname = 'Node %d covered'%i) 
#model.parameters.timelimit=3600
#
#solution = model.solve(log_output = True)
#toc=time.time()
#print("Time taken for solving is " + str((toc-tic)) + " sec")
#width = 2*colNumber - 2
#height = 2*rowNumber - 2
#plt.figure(figsize=(width, height))
#plt.xlabel("Coordinate X")
#plt.ylabel("Coordinate Y")
#plt.title("Solution of DP/SP for Multi-agent EECPP Problem")
#
#for n in NodesAndDeparturePoint:
#    if n!=departurePoint:
#        plt.scatter(x=coord_x[n],y=coord_y[n],color='blue')
#        currentAxis=plt.gca()
#        rect=patches.Rectangle( (coord_x[n]-1/2*sideLength,coord_y[n]-1/2*sideLength),sideLength,sideLength,linewidth=1,edgecolor='k',facecolor='none' )
#        currentAxis.add_patch(rect)
#    else:
#        plt.scatter(x=coord_x[n],y=coord_y[n],color='green')
#        currentAxis=plt.gca()
#        rect=patches.Rectangle( (coord_x[n]-1/2*sideLength,coord_y[n]-1/2*sideLength),sideLength,sideLength,linewidth=1,edgecolor='k',facecolor='none' )
#        currentAxis.add_patch(rect)
#
##for obstacles, first plot in red to mark
#for obstacle in obstacles:
#    # use x0,y0 to be short for coord_x[obstacle],coord_y[obstacle]
#    x0=coord_x[obstacle]
#    y0=coord_y[obstacle]
#    plt.scatter(x=x0,y=y0,color='red')
#    #second, pathes.Rectangle
#    currentAxis=plt.gca()
#    rect=patches.Rectangle( (x0-1/2*sideLength,y0-1/2*sideLength),sideLength,sideLength,linewidth=1,edgecolor='k',facecolor='grey' )
#    currentAxis.add_patch(rect)
##end
#    
#sets=[]
#agent1_set=[]
#agent2_set=[]
#agent3_set=[]
#agent4_set=[]
#agent5_set=[]
#agent6_set=[]
#agent7_set=[]
#agent8_set=[]
#agent9_set=[]
#agent10_set=[]
#for l in labels:
#    if x[l].solution_value > 0.9:
#        sets.append(all_qualified_labels[l])
#agent1_set=sets[0][:-1]
#if len(sets) >=2:
#    agent2_set=sets[1][:-1]
#if len(sets)>=3:
#    agent3_set=sets[2][:-1]
#if len(sets)>=4:
#    agent4_set=sets[3][:-1]
#if len(sets)>=5:
#    agent5_set=sets[4][:-1]
#if len(sets)>=6:
#    agent6_set=sets[5][:-1]
#if len(sets)>=7:
#    agent7_set=sets[6][:-1]
#if len(sets)>=8:
#    agent8_set=sets[7][:-1]
#if len(sets)>=9:
#    agent9_set=sets[8][:-1]
#if len(sets)>=10:
#    agent10_set=sets[9][:-1]
#length = len(agent1_set)
#for i in range(length-1):
#    start = agent1_set[i]
#    end   = agent1_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='blue')
#
#length = len(agent2_set)
#for i in range(length-1):
#    start = agent2_set[i]
#    end   = agent2_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='orange')
#
#length = len(agent3_set)
#for i in range(length-1):
#    start = agent3_set[i]
#    end   = agent3_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='green')
#    
#length = len(agent4_set)
#for i in range(length-1):
#    start = agent4_set[i]
#    end   = agent4_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='red')
#
#length = len(agent5_set)
#for i in range(length-1):
#    start = agent5_set[i]
#    end   = agent5_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='purple')
#
#length = len(agent6_set)
#for i in range(length-1):
#    start = agent6_set[i]
#    end   = agent6_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='brown')
#
#length = len(agent7_set)
#for i in range(length-1):
#    start = agent7_set[i]
#    end   = agent7_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='pink')
#    
#length = len(agent8_set)
#for i in range(length-1):
#    start = agent8_set[i]
#    end   = agent8_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='gray')
#    
#length = len(agent9_set)
#for i in range(length-1):
#    start = agent9_set[i]
#    end   = agent9_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='olive')
#
#length = len(agent10_set)
#for i in range(length-1):
#    start = agent10_set[i]
#    end   = agent10_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='cyan')
#plt.savefig("figure.pdf")
#plt.show() 