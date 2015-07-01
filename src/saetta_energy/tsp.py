#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import pulp
import numpy as np

cities = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P']
distances = [
    #1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16
    [0, 509, 501, 312, 1019, 736, 656, 60, 1039, 726, 2314, 479, 448, 479, 619, 150], #1
    [509, 0, 126, 474, 1526, 1226, 1133, 532, 1449, 1122, 2789, 958, 941, 978, 1127, 542], #2
    [501, 126, 0, 541, 1516, 1184, 1084, 536, 1371, 1045, 2728, 913, 904, 946, 1115, 499], #3
    [312, 474, 541, 0, 1157, 980, 919, 271, 1333, 1029, 2553, 751, 704, 720, 783, 455], #4
    [1019, 1526, 1516, 1157, 0, 478, 583, 996, 858, 855, 1504, 677, 651, 600, 401, 1033], #5
    [736, 1226, 1184, 980, 478, 0, 115, 740, 470, 379, 1581, 271, 289, 261, 308, 687], #6
    [656, 1133, 1084, 919, 583, 115, 0, 667, 455, 288, 1661, 177, 216, 207, 343, 592], #7
    [60, 532, 536, 271, 996, 740, 667, 0, 1066, 759, 2320, 493, 454, 479, 598, 206], #8
    [1039, 1449, 1371, 1333, 858, 470, 455, 1066, 0, 328, 1387, 591, 650, 656, 776, 933], #9
    [726, 1122, 1045, 1029, 855, 379, 288, 759, 328, 0, 1697, 333, 400, 427, 622, 610], #10
    [2314, 2789, 2728, 2553, 1504, 1581, 1661, 2320, 1387, 1697, 0, 1838, 1868, 1841, 1789, 2248], #11
    [479, 958, 913, 751, 677, 271, 177, 493, 591, 333, 1838, 0, 68, 105, 336, 417], #12
    [448, 941, 904, 704, 651, 289, 216, 454, 650, 400, 1868, 68, 0, 52, 287, 406], #13
    [479, 978, 946, 720, 600, 261, 207, 479, 656, 427, 1841, 105, 52, 0, 237, 449], #14
    [619, 1127, 1115, 783, 401, 308, 343, 598, 776, 622, 1789, 336, 287, 237, 0, 636], #15
    [150, 542, 499, 455, 1033, 687, 592, 206, 933, 610, 2248, 417, 406, 449, 636, 0], #16
]

def tsp_problem(cities, distances):
    # calculating costs
    costs = pulp.makeDict([cities, cities], distances, 0)

    # calculating routes
    routes = []
    for i in cities:
        for j in cities:
            if(i != j):
                routes.append((i,j))

    # creating problem
    prob = pulp.LpProblem('Travelling Salesman Problem', pulp.LpMinimize)
    x = pulp.LpVariable.dicts('route', (cities, cities), lowBound=0, upBound=1, cat=pulp.LpInteger)
    u = pulp.LpVariable.dicts('u', cities[1:], lowBound=2, upBound=len(cities), cat=pulp.LpInteger)

    # objective function is added to prob first
    prob += sum([x[w][b]*costs[w][b] for (w, b) in routes]), 'Sum_of_Tour_Costs'

    # in constraints
    for i in cities:
        tmp = []
        for j in cities:
            if(i != j):
                tmp.append(x[i][j])

        prob += sum(tmp) == 1, 'route_out_%s' % i

    # out constraints
    for i in cities:
        tmp = []
        for j in cities:
            if(i != j):
                tmp.append(x[j][i])
        prob += sum(tmp) == 1, 'route_in_%s' % i

    # dummy constraints
    for i in cities:
        for j in cities:
            if((i != j) and ((i != cities[0]) and (j != cities[0]))):
                prob += u[i] - u[j] + 1 <= (len(cities) - 1) * (1 - x[i][j])


    # (optional) write problem
    #prob.writeLP('tsp.lp')

    # solve problem
    prob.solve()

    # results
    tsp_route = [cities[0] for _ in range(len(cities))]
    total_cost = prob.objective.value()

    # extract tsp_route
    for v in prob.variables():
        if v.name.startswith('u_'):
            idx = int(v.varValue) - 1
            city = v.name.split('u_')[1]

            tsp_route[idx] = city

    #print('Status: %s' % pulp.LpStatus[prob.status])

    return tsp_route, total_cost, prob
