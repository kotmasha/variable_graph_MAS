#/bin/bash python3
import numpy as np
class agentask:
    def __init__(self,graph,agents):
        self.graph=graph
        self.agents=agents
        self.taskList={}
        for name in self.graph.names:
            self.taskList[name] = {}
            if name in self.agents:
                if 'Target' in self.agents[name]['Task']: #checks if the agent has a target
                    self.taskList[name]['target']=np.array(self.agents[name]['Task']['Target'])
                self.taskList[name]['keepUpQ']=self.agents[name]['Task']['KeepUpQ']
            else:
                self.taskList[name]['target']=None # May want to remove this line
                self.taskList[name]['keepUpQ']=True
        
 