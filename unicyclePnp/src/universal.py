#/bin/bash python3

import numpy as np

def col2tup(col):
    print(col)
    return tuple(*col.T.tolist())

