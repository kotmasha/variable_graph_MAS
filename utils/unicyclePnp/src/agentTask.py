#/bin/bash python3
import numpy as np
class agentask:
    def __init__(self,graph,leaders,uniAgent):
        self.graph=graph
        self.leaders=leaders
        self.constLeader=list(self.leaders.keys())[0]
        self.uniAgent=uniAgent
        self.taskList={}
        if self.uniAgent:
            for name in self.graph.names:
                self.taskList[name] = {}
                tmp=np.array([[self.leaders[self.constLeader]['Target'][0]],[self.leaders[self.constLeader]['Target'][1]]])
                self.taskList[name]['target']=tmp
                self.taskList[name]['keepUpQ']=self.leaders[self.constLeader]['keepUpQ']
        else:
            for name in self.graph.names:
                self.taskList[name] = {}
                if name in self.leaders:
                    tmp=np.array([[self.leaders[name]['Target'][0]],[self.leaders[name]['Target'][1]]])
                    self.taskList[name]['target']=tmp
                    self.taskList[name]['keepUpQ']=self.leaders[name]['keepUpQ']
                else:
                    self.taskList[name]['target']=None
                    self.taskList[name]['keepUpQ']=True
        
 