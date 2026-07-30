import numpy as np

# Add skewJ, rotation matrices, flip, fliptwo, etc. here
# the flip matrices are called /flip and /fliptwo in the overleaf
flip=np.matrix([[-1,0],[0,1]])  # Flip around X axis
fliptoo=np.matrix([[0,1],[1,0]])# Swap X and Y axes
skewJ=np.matrix([[0,-1],[1,0]]) # 90 degrees CCW rotation