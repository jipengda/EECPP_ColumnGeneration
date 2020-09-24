# -*- coding: utf-8 -*-
# This is full dynamic programming(or complete dynamic programming)
"""
Created on Wed Aug  5 21:26:51 2020

@author: Jipeng
"""

"""
Or complete dynamic programming
"""
"""
Refer incomplte_DP.py for reference
this intends to get the minimum reduced cost label candidate during each iteration
which is different from incomplete_DP.py(it can generate just need to any one with negative reduced cost)
"""

# the following part is borrowed from obtain_labels_data.py
# add new parameter duals, duals is dual values of constraints froom main function.
import time
import numpy as np
import math
number_of_criteria_based_on_adjacent_nodes=0
number_of_criterion1_eliminated=0
number_of_optimalityPrinciple_eliminated=0
colNumber = 4
rowNumber = 5
distance_lambda = 0.1164
turn_gamma = 0.0173


# distance_lambda = 0.1164 kj/m
# turn_gamma = 0.0173 kj/deg
# totalCost renamed to reduced cost
# It is not engouth, we also need defeine totalCost individually.
def obtain_one_candidate(q,distance,D,duals, coord_x, coord_y, nodesNumber, Battery_capacity_constraint, departurePoint, obstacles):
# this function obtain_one_candidate intends to get an one_candidate which hope to optimize master model
# this function don't will return all_quailied_labels, which is not the requirement.
    global distance_lambda
    global turn_gamma
    start_time = time.time()
    # reducedCost & totalCost
    # reducedCost for reducedCost constraint, totalCost for totalCost constraint.
    reducedCost = 0
    totalCost = 0
    lastNode=[]
    second_to_lastNode=[]
    iteration = nodesNumber
    sets=[]
    one_candidate=[]
    # defeine all_candidates to store all qualified labels
    all_candidates=[]
    # define minimum_label to returen the minimum label
    minimum_label = []
    all_qualified_labels=[]
    nodes=[node for node in range(0, nodesNumber) if node!= departurePoint and node not in obstacles] # new node to be added
    for newNode in nodes:
        # reducedCost & totalCost
        # but be careful, don't mess this two up
        reducedCost = 0
        totalCost = 0
        feasible_set=[departurePoint]
        lastNode=departurePoint
        second_to_lastNode=departurePoint # This is special case for k=1 iteration
        pass0 = Elimination_criteria_based_on_obstacle(obstacles, lastNode, newNode)
        if pass0 is True:
            pass1 = Battery_capacity_limit_check(q, distance, second_to_lastNode, lastNode, newNode, coord_x, coord_y, D , totalCost, Battery_capacity_constraint )
            if pass1 is True:
                feasible_set.append(newNode)
                feasible_label = list(feasible_set)
                go=lastNode
                to=newNode
                turnCost=0
#                distanceCost=distance_lambda*D[go][to]
                distanceCost = distance_lambda * distance[go, to]
                totalCost = turnCost + distanceCost
                routeCost = duals[departurePoint] * 1
                reducedCost=turnCost+distanceCost - routeCost
                label=[totalCost]
                feasible_set.append(label)
                sets.append(feasible_set)
                pass0 = Elimination_criteria_based_on_obstacle(obstacles, newNode, departurePoint)
                if pass0 is True:
                    permission = Battery_capacity_limit_check(q,distance, lastNode, newNode, departurePoint, coord_x, coord_y, D, totalCost, Battery_capacity_constraint)
                    if permission is True:
#                        turnCost = turn_gamma * angle(lastNode, newNode, departurePoint, coord_x, coord_y)
                        turnCost=q[lastNode, newNode, departurePoint]
                        distanceCost=distance_lambda * D[newNode][departurePoint]
                        totalCost = totalCost + turnCost + distanceCost
                        routeCost = duals[newNode] * 1
                        reducedCost=reducedCost+turnCost + distanceCost - routeCost 
                        # how to store reducedCost
                        label=[totalCost]

                        # first add the label,second judge the label.
                        # if reduced cost of the label qualifies, return and end the function.
                        feasible_label.append(departurePoint)
                        feasible_label.append(label)
                        all_qualified_labels.append(feasible_label)
                        # define a judgement "reducedCost_check function" for reducedCost constraint
                        # if reducedCost<0, return True. if reducedCost >= 0, return False.
                        pass1 = reducedCost_check(reducedCost)
                        if pass1 is True:
                            one_candidate = list(feasible_label)
                            one_candidate.append(reducedCost)
                            all_candidates.append(one_candidate)
                            # we dont need reutrn one_candidate
#                            return one_candidate
                        else:
                            pass
               
    # node in start end
    label=[]
    # update thanks to new requirement: ending point must be exactly same with leaving point
    # nodeNumber -2(previously -3) nodes is added here
    for k in range(1, iteration-1):
        flag = 0 # flag work as flag, once at the iteration, no new node is eligible to be added, break(way: return) the loop to save time
        sets_copy=list(sets)
        sets=[]
        for S_0 in sets_copy:
            second_to_lastNode = S_0[-3]
            lastNode = S_0[-2]
            S=S_0.copy()
            S.pop()
            for newNode in nodes: # nodes replace nodesAndArrival to improve efficiency
                label=[]
                reducedCost = 0
                feasible_set = list(S_0) # Make a shallow copy # It is not good enough now because S includes label at last posion
                label=feasible_set.pop() #Added thanks to the previous line
                # totalCost
                # but where is the reducedCost when two or more nodes
                totalCost = label[0]
                pass0 = Elimination_criteria_based_on_obstacle(obstacles, lastNode, newNode)
                if pass0 is True:
                    pass1 = Battery_capacity_limit_check(q,distance,second_to_lastNode, lastNode, newNode, coord_x, coord_y, D , totalCost, Battery_capacity_constraint )
                    if pass1 is False:
                        pass
                    else:
                        pass2 = Elimination_criteria_based_on_the_set_S_only(S, newNode)
                        if pass2 is False:
                            pass
                        else:
                            flag = 1
                            feasible_set.append(newNode)
                            feasible_label = list(feasible_set)
                            go=lastNode
                            to=newNode
#                            turnCost=turn_gamma * angle(second_to_lastNode, go, to, coord_x, coord_y)
                            turnCost=q[second_to_lastNode,go,to]
#                            distanceCost=distance_lambda * D[go][to]
                            distanceCost= distance_lambda * distance[go, to]
                            # reducedCost
                            # define a function calculating sum of route cost in label
                            totalCost = totalCost + turnCost + distanceCost
                            label = [totalCost]
                            feasible_set.append(label)
                            sets.append(feasible_set)
        sets=optimalityPrinciple(sets)
        for feasible_set in sets:
            lastNode = feasible_set[-2]
            second_to_lastNode = feasible_set[-3]
            newNode = departurePoint
            # reducedCost
            totalCost = feasible_set[-1][0]
            pass0 = Elimination_criteria_based_on_obstacle(obstacles, lastNode, newNode)
            if pass0 is True:
                permission = Battery_capacity_limit_check(q,distance, second_to_lastNode, lastNode, newNode, coord_x, coord_y, D, totalCost, Battery_capacity_constraint)
                if permission is True:
#                    turnCost = turn_gamma * angle(second_to_lastNode, lastNode, newNode, coord_x, coord_y)
                    turnCost = q[second_to_lastNode, lastNode, newNode]
#                    distanceCost = distance_lambda * D[lastNode][newNode]
                    distanceCost = distance_lambda * distance[lastNode, newNode]
                    # reducedCost
                    totalCost = feasible_set[-1][0] + turnCost + distanceCost
                    label = [totalCost]
                    # first add the label, second judge the label.
                    # if reduced csot of the label qualifies, return and end the function.
                    feasible_set_copy = list(feasible_set[:-1])
                    feasible_set_copy.append(newNode)
                    feasible_set_copy.append(label)
                    all_qualified_labels.append(feasible_set_copy)
                    # defeine a judgement "reducedCost_check function" for reduced cost constraint
                    # if reducedCost <0, return True. if reducedCost >=0, return False.
                    # we define a "reducedCost_calculate function" for reduced cost calculation
                    reducedCost = reducedCost_calculate(totalCost, feasible_set_copy, duals)
                    pass1 = reducedCost_check(reducedCost)
                    if pass1 is True:
                        one_candidate = list(feasible_set_copy)
                        one_candidate.append(reducedCost)
                        all_candidates.append(one_candidate)
                        # we don't need return one_candidate
#                        return one_candidate
                    else:
                        pass

        if flag == 0:
            minimum_label = min_reduced_label(all_candidates)
            return minimum_label
            #we dont need return one_candidate
#            return one_candidate
        # Time limit to 3600s
        currentTime = time.time()
        differentTime = currentTime - start_time
        if differentTime >= 3600:
            minimum_label = min_reduced_label(all_candidates)
            return minimum_label
            #we dont need return one_candidate
#            return one_candidate
        # end
    minimum_label = min_reduced_label(all_candidates)
    return minimum_label
    #we dont need return one_candidate
#    return one_candidate

def Elimination_criteria_based_on_obstacle(obstacles, lastNode, newNode):
    global colNumber
    sideLength = 1
    flag=1 # initialize flag=1
    for obstacle in obstacles:
        x3 = obstacle % colNumber
        y3 = obstacle // colNumber
        r_xmin = x3 - sideLength/2
        r_xmax = x3 + sideLength/2
        r_ymin = y3 - sideLength/2
        r_ymax = y3 + sideLength/2
        if RSIntersection(r_xmin, r_xmax, r_ymin, r_ymax, lastNode, newNode) == 1:
            flag = 0 #flag is variable to replace directly return False of True because we need to test all obstacles
        else:
            flag = 1
        if flag == 0:#once flag change to 0, means one obstacle is hit, thus new node can not be added
            return False
    return True# This means all obstacles are not hit, new node can be considered to be added
"""
 * if [x1, x2] abd [x3, x4] (x4 maybe smaller than x3) has interrection or has not，if yes: return 1， if no: return 0
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
def RSIntersection(r_xmin,r_xmax,r_ymin, r_ymax, nodeA, nodeB):
    global colNumber
    global rowNumber
    A_x = nodeA % colNumber
    A_y = nodeA // colNumber
    B_x = nodeB % colNumber
    B_y = nodeB // colNumber
    if (A_y == B_y):# 线段平行于x轴
        if A_y <= r_ymax and A_y >= r_ymin:
            return IntervalOverlap(r_xmin, r_xmax, A_x,B_x)
        else:
            return 0

	# AB两点交换，让B点的y坐标最大

    # Exchange node A and node B, let B's y value is bigger
    t = 0
    if A_y > B_y:
       t = A_y
       A_y = B_y
       B_y = t
       t= A_x
       A_x = B_x
       B_x=t
	
    # In xianduan AB, to find point C and D
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

#As the algorithm is based on dynamic programming, it is useful to limit the number of states created
# For a state (S,i) to which node j is to be added, the frist four elimination tests described for state(SU{j},j) are independent of
# the terminal node i
# while the next four tests depend on the labels in H(S,i)
# To simplify the execution of the tests, information on the states is stored in  a three level structure
# At the first level, we have the set S and its load y(S)
# At the second level,for the given set S, we have the terminal nodes i belongs to S used to form the states (S,i)
# At the third level, for a state (S,i), the reduced set of lables H(S,i) is stored.
def Battery_capacity_limit_check(q,distance, second_to_lastNode, lastNode, newNode, coord_x, coord_y, D , totalCost, Battery_capacity_constraint ):
    global distance_lambda
    global turn_gamma
    futureCost=0
    go=lastNode
    to=newNode
#    turnCost=turn_gamma * angle(second_to_lastNode, go, to, coord_x, coord_y)
    turnCost=q[second_to_lastNode,go,to]
    #judge turnCost nan or not nan
    flag = math.isnan(turnCost)
    #end
    if flag == True:
        turnCost=0
    else:
        turnCost=turnCost
#    distanceCost=distance_lambda * D[go][to]
    distanceCost=distance_lambda * distance[go,to]
    totalCost=totalCost+turnCost + distanceCost
    futureCost = totalCost
    if futureCost > Battery_capacity_constraint:
        return False
    return True

def Elimination_criteria_based_on_the_set_S_only(S,newNode):
    global number_of_criterion1_eliminated
    j = newNode
    S = S # we have the set S
    # criterion 1
    if j in S: # criterion #1, new node should not have been visited
        number_of_criterion1_eliminated+=1
        return False
    # criterion 1 end
    return True 

def optimalityPrinciple(sets): # it needs to change because sets each set has a label as last element, this is different from before
    global number_of_optimalityPrinciple_eliminated
    the_nodes_of_R=[]
    sets_copy=sets.copy()
    R = []
    for R in sets:
        the_nodes_of_R = set(R[:-1]) #[:-1] is added now because we don't want label as node
        for R_2 in sets_copy:
            if R_2!=R:
                if set(R_2[:-1]) == the_nodes_of_R:
                    if R_2[-2] == R[-2]:
                        if R_2[-3] == R[-3]:
                            if R_2[-1][0]<=R[-1][0]:
                                number_of_optimalityPrinciple_eliminated+=1
                                sets_copy.remove(R) #The label is eliminated if there exists another label with both a lesser time and a lesser cost
                                break
    return sets_copy

def distance(firstNode, secondNode,coord_x,coord_y):
    i=firstNode
    j=secondNode
    distanceValue=0
    distanceValue=np.hypot(coord_x[i]-coord_x[j],coord_y[i]-coord_y[j])
    return distanceValue

def angle(second_to_lastNode, lastNode, newNode, coord_x, coord_y):
    o=second_to_lastNode
    p=lastNode
    q=newNode
    radians_to_degrees = 180/(math.pi)
    theta_radians=0
    theta_degrees=0
    distance_o_p=distance(o,p,coord_x,coord_y)
    distance_p_q=distance(p,q,coord_x,coord_y)
    distance_o_q=distance(o,q,coord_x,coord_y)
    theta_radians=math.pi-np.arccos(round((distance_o_p**2+distance_p_q**2-distance_o_q**2)/(2*distance_o_p*distance_p_q),2))
    theta_degrees=theta_radians * radians_to_degrees
    return theta_degrees


def reducedCost_check(reducedCost):
    if reducedCost < 0:
        return True
    elif reducedCost >= 0:
        return False
def reducedCost_calculate(totalCost, feasible_set_copy, duals):
    reducedCost = 0
    reducedCost = totalCost
    #:-2 is becuase we don't need calculate duals of last two elements, of which one is total cost, another one is departurepoint.
    # for departurepoint, we already calculate it we don't need calculate it again.
    
    nodes = feasible_set_copy[:-2]
    for node in nodes:
        reducedCost = reducedCost - duals[node] * 1
    return reducedCost

def min_reduced_label(all_candidates):
    minimum_label=[]
    # defeine minimum_cost for comparing reduced cost between two candidates
    minimum_cost = 1
    for candidate in all_candidates:
        if candidate[-1]<minimum_cost:
            minimum_cost = candidate[-1]
            minimum_label = candidate
        
    return minimum_label