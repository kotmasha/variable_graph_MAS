# PnP_multi-agent_navigation


## Dependencies
This MATLAB package relies on Matt Jacobson's lcon2vert function package for handling polytopes. In order for the main .m file to run, you will need to include the ./polytopes/ subdirectory in the search tree.


## Purpose
The provided code is an implementation of the PnP controller, as described in Section V of our paper "Plug-and-Play Cooperative Navigation: From Single-Agent Navigation Fields to Graph-Maintaining Distributed MAS Controllers". As such, it is only meant to cover planar sphere worlds and planar topological sphere worlds with star-convex obstacles. Moreover, the only implemented communication graphs are the N-path and the N-cycle (with some tweaks to the code).


## INITIALIZATION
The simulation may be run with different initial parameter values, as documented in lines 5--75 in the code. While some of the code may apply in higher dimension, the code as is will only work in the plane, that is: the line
  stateDim=2;
is not something you can easily alter without writing new code.


## Executable code
The meaningful simulation parameters, assuming you keep the same geometry settings, are set in lines 18--25.

A) Lines 31--110 set up the geometric environment (the workspace and obstacles).

B) Lines 111--135 set up the initial MAS configuration.

C) Lines 136--212 set up the requisite elements of the computation (the adjacency matrix in particular, as well as some parameters). 

D) Lines 213--229 runs the ODE solver using the HyEQ package, in anticipation of future hybrid versions of the PnP controller.

E) Lines 230--248 removes extraneous data from the solver's output and saves the remaining data.

F) Lines 249--322 creates the main figure, in which the simulation video will be processed. This can be run immediately after loading/generating the HyEQ data.

G) Lines 323--352 completes the main figure with a plot of normalized edge lengths for the different edges of the network.

H) Lines 353--366 generates a separate normalized edge lengths figure (used in the paper)

I) Lines 367--394 generates a figure showing individual agent tracks (used in the paper)

J) Lines 395--419 generates a figure showing progressive states of the network over time.

K) Lines 420--464 generates the simulation video and saves it.


## Functions

Are rather well-documented in the code, following the detailed development in Section V of the paper. However, if you would like to alter the shapes of the obstacles, you need to keep in mind the following (see details in Section V of the paper):

- the number of obstacles needs to be provided in advance as a global parameter (obstacleNum);

- each obstacle must be provided together with a center (global obstacleCenters) relative to which it is star-convex and the radius of a ball about that center (global obstacleRadii), which is contained in the interior of the obstacle;

- the boundary of each obstacle must be provided in the function barrierCurve (lines 682--698), in polar coordiantes (in the form $\rho=r(\theta)$), together with the derivative $d\rho/d\theta$ that must be provided in the function barrierCurveDeriv (lines 700--716);

- each obstacle needs to be provided with a clearance value (global obstacleClearances), to make sure that the inflated obstacles do not intersect;

- the global values are set in advance (currently in section A of the code); the obstacle shapes are set in the barrierCurve functions themselves.



# Provided Data Sets

A number of data sets are provided under the /data/ sub-directory, which resulted from interesting simulation runs. Each data set may be loaded, and then visualized by the provided package, as follows:

- load the data set in MATLAB

- Run section F

- Run section G

- (optional) Run sections H/I/J

- Run section K to produce a video.

