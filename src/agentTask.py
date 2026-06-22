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
                self.taskList[name]['target']=np.array(self.leaders[name]['Task']['Target']) # DWR 6/15/26: Not sure why it had separate ['Target'][0] and [1]
                self.taskList[name]['keepUpQ']=self.leaders[name]['Task']['KeepUpQ']
            else:
                self.taskList[name]['target']=None
                self.taskList[name]['keepUpQ']=True
        
 