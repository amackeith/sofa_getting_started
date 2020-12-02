import numpy as np
import matplotlib.pyplot as plt
import sys, os


def get_nodes(filename):
    nodes = []
    with open(filename) as f:
        
        x = f.readline()
        i = 0
        
        while x != "$Nodes\n":
            i += 1
            x = f.readline()
        x = f.readline()
        x = f.readline()
        
        while x != "$EndNodes\n":
            # print x
            x = x.split()
            nodes.append(np.array([float(x[1]), float(x[2]), float(x[3])]))
            x = f.readline()
            
    return nodes


def all_nodes_within_and_below_zero(filename, radius):
    nodes = get_nodes(filename)
    inds = []
    for i,nd in enumerate(nodes):
        if np.sqrt(nd[0]**2 + nd[1]**2) < radius and nd[2] < 0:
            inds.append(i)
            
    return inds

def nth_smallest_point(filename, n=0):
    
    nodes = get_nodes(filename)
    nodes = np.array(nodes)
    distances = []
    for i in nodes:
        d = np.linalg.norm(i)
        distances.append(d)

    distances_cp = distances[:]
    distances_cp.sort()
    smallest_ind = distances.index(distances_cp[0])
    second_smallest_ind = distances.index(distances_cp[1])
    #print nodes[smallest_ind], nodes[second_smallest_ind], distances_cp[0:3], smallest_ind, second_smallest_ind
    
    nth_sm_ind = distances.index(distances_cp[n])
    return nodes[nth_sm_ind], nth_sm_ind

