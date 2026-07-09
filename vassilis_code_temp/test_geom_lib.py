import numpy as np
import shapely as sp
import polygeom_lib as pg
from shapely.geometry import Point,Polygon,LinearRing,LineString
outer=LinearRing([[0,0],[0,4],[7,4],[7,0]])
poly=LinearRing([[1,1],[2,2],[1,3],[5,3],[4,2],[5,0.5]])
tree=pg.polytriangulation(poly,outer,False)
tree2=pg.polyconvexdecomposition(poly,outer,False)
