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
    def __init__(self, names, edges=None):
        #ingest the vertex names and enumerate them
        self.names=list(names)
        if edges==None:
            self.graph=np.zeros((0,0))
            self.edges=np.zeros((0,0))
            self.vertexIndices={names[0]:0}
        else:
            self.vertexIndices = {vertex: idx for idx, vertex in enumerate(names)}
            self.indexToVertex={v: k for k, v in self.vertexIndices.items()} # DWR 6/16/26: used for visualizations in network.py

            self.edges = [(self.vertexIndices[edge[0]], self.vertexIndices[edge[1]]) for edge in edges]
            rows, cols = zip(*self.edges)
            data = np.ones(len(self.edges), dtype=int)
            self.graph = csr_matrix((data, (rows, cols)), shape=(len(names), len(names)))

    def agentNum(self):
        return len(self.names)
    
    def idxToNames(self, ls): #return the list of vertex names corresponding to a list of indices
        return [(None if idx == -9999 else self.names[idx]) for idx in ls]
    
    def namesToIdx(self,nameList):
        return [self.vertexIndices[name] for name in nameList]
    
    def neighbors(self, name):
        # returns a list of neighbors for a given agent name
        if self.edges is None or len(self.edges) == 0:
            return []
        else:
            ls = enumerate(self.graph.getrow(self.vertexIndices[name]).toarray()[0].tolist())
            return [self.names[x] for x, y in ls if y == 1]
    
    def dfs(self, start_vertex=0):
        dfs_order, dfs_preds = depth_first_order(self.graph, start_vertex, return_predecessors=True)
        return self.idxToNames(dfs_order), self.idxToNames(dfs_preds)

    def addEdge(self,name1,name2):
        # add an edge to the graph: update self.edges, update self.graph in a manner consistent with the procedure in the constructor
        return None
    
    def removeEdge(self,name1,name2):
        # remove an edge from the graph: update self.edges, update self.graph in a manner consistent with the procedure in the constructor
        return None

    def addVertex(self,name):
        # add an vertex to the graph: update self.Names, update self.graph in a manner consistent with the procedure in the constructor
        return None
    
    def removeVertex(self,name):
        # remove a vertex and all its edges from the graph: update self.edges, update self.graph in a manner consistent with the procedure in the constructor
        return None