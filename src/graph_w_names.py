import sys
import os
import math
from numpy import linalg as la
import numpy as np
import random
from scipy.sparse import csr_matrix
# from scipy.sparse.csgraph import connected_components
from scipy.sparse.csgraph import depth_first_order
import universal

class graph_w_names:
    def __init__(self,names,edges):
        self.names=names 
        self.edges=edges
        self.vertex_indices={vertex:idx for idx,vertex in enumerate(names)}
        edges_indices = [(self.vertex_indices[edge[0]], self.vertex_indices[edge[1]]) for edge in edges]
        # Use edges_indices instead of rows and cols
        data = np.ones(len(edges_indices), dtype=int)
        self.graph = csr_matrix((data, zip(*edges_indices)), shape=(len(names), len(names)))
     
    def idxToNames(self,ls):
        return [(None if idx==-9999 else self.names[idx]) for idx in ls]
    
    def neighbors(self,name):
        # returns a list of neighbors for a given agent name
        ls=enumerate(self.graph.getrow(self.vertex_indices[name]).toarray()[0].tolist())
        return [self.names[x] for x,y in ls if y==1]

    def dfs(self, start_vertex=0):
        dfs_order,dfs_preds = depth_first_order(self.graph, start_vertex, return_predecessors=True)
        return self.idxToNames(dfs_order),self.idxToNames(dfs_preds)
    