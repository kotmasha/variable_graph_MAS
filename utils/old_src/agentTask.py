#/bin/bash python3
import numpy as np
class agentask:
    def __init__(self,graph,leaders):
        self.graph=graph
        self.leaders=leaders
        self.taskList={}
        for name in self.graph.names:
            self.taskList[name] = {}
            if name in self.leaders:
                tmp=np.array([[self.leaders[name]['Target'][0]],[self.leaders[name]['Target'][1]]])
                self.taskList[name]['target']=tmp
                self.taskList[name]['keepUpQ']=self.leaders[name]['keepUpQ']
            else:
                self.taskList[name]['target']=None
                self.taskList[name]['keepUpQ']=True
        
 