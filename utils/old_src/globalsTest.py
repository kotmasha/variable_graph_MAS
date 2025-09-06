#/bin/bash python3

#System imports - not always used but necessary sometimes
import sys
import os
import math
import inspect
import agent
from agent import baseAgent

def get_subclasses(module, base_class):
    subclasses = []
    for name, obj in inspect.getmembers(module):
        if inspect.isclass(obj) and issubclass(obj, base_class) and obj != base_class:
            subclasses.append(obj)
    return subclasses

# Example usage
subclasses = get_subclasses(agent, baseAgent)  # Replace BaseClass with your base class
x=subclasses[0].navSphere(subclasses[0])


