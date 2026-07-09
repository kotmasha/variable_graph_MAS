#!/usr/bin/env python

"""
MIT License (modified)

Copyright (c) 2020 The Trustees of the University of Pennsylvania
Authors:
Omur Arslan <omur@seas.upenn.edu>
Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

FURTHER UPDATES BY Dan P. Guralnik:
- using shapely functionality more efficiently

"""

from __future__ import division
from collections import deque
#from hashlib import _BlakeHash
import numpy as np
#import shapely as sp
import math
import tripy
from shapely.geometry import Point,Polygon,LinearRing,LineString
from shapely.ops import unary_union,orient
from operator import itemgetter

def ringsignedarea(lring):
    """
    ringsignedarea(lring) provides the signed area enclosed by a [not necessarily valid] shapely.geometry.LinearRing
    """
    area=0.0
    for idx in range(len(lring.coords)-1):
        area=area+0.5*np.cross(lring.coords[idx],lring.coords[idx+1])
    return area

def polysignedarea(poly):
    """
    polysignedarea(poly) determines the signed area of a VALID shapely.geometry.Polygon

    Input:
        poly    : a shapely.geometry.Polygon

    Output:
        signed area of the polygon poly, according to the orientation of the outside ring
    """
    polysign=(-1)**(1+int(poly.exterior.is_ccw))
    polytemp=orient(poly)
    return polysign*(ringsignedarea(polytemp.exterior)+sum([ringsignedarea(item) for item in polytemp.interiors]))

def inpolygon(poly, pts):
    """
    inpolygon(xy, p) determines if a given set of points, p, are contained in 
    a polygon, with vertex set xy.
    Input:  
        poly : A shapely.geometry.Polygon valid polygon
        pts  : A list of shapely.geometry.Point objects
    Output: 
        A boolean array indicating which points are contained in the polygon 
    """
        
    return [poly.contains(pt) for pt in pts]    


def polydist(poly, pts):
    """
    polydist(poly, pts) computes the distances of a list, pts, of shapely.geometry.Points to the boundary 
    of a valid shapely.geometry.Polygon, poly, and produces points on the polygon boundary closest to
    the provided points in pts, in the same order.   

    Input:  
        poly : A valid shapely.geometry.Polygon object 
        pts  : A list of shapely.geometry.Point objects
    Output: 
        D  : Distance between points and the polygon 
        C  : Coordinates of points on the polygon boundary closest to the input points
    """
    if np.size(pts)==0:  #if no points are provided, then distances/projections lists are empty
        return np.array([]),np.array([])
    elif poly.is_empty:   # Distance to empty set is infinity
        D = np.zeros(len(pts))
        D.fill(np.inf)
        C = np.zeros((len(pts),2))
        C.fill(np.inf) 
        return D,C
    else:
        # Compute distances and closest points on the polygon boundary
        D = np.array([pt.distance(poly.boundary) for pt in pts])
        C = [poly.boundary.interpolate(poly.boundary.project(pt)) for pt in pts]  
    
    return D,C  


def polyxhplane(poly, bpt, nvec):
    """ (DG)
    Computes the intersection of a shapely.geometry.Polygon, poly and 
    the halfplane determined by the boundary point bpt and the inward 
    normal nvec.
    
    Input:
        poly    : A shapely.geometry.Polygon
        bpt     : A shapely.geometry.Point on the boundary of the halfplane
        nvec    : An inward normal of the halfplane as a (1x2) numpy.array
    Output:
        The required intersection, provided as a shapely.geometry.Polygon
        ***Note that this function may return a shapely Collection if the input
        ***polygon is not convex.
    """
    nvec_norm=np.linalg.norm(nvec)
    if (nvec_norm == 0):
        raise ValueError('Normal vector to a line should not vanish')   
    else:
        #form a tangent vector to the line
        bptnp=np.array(bpt.coords[0])
        vec=np.array([-nvec[1],nvec[0]])/nvec_norm

        #to form a line segment through bpt, long enough to contain all intersections, compute
        HD=bpt.hausdorff_distance(poly.boundary)+1

        #form a rectangle containing poly and contained in the halfspace
        rect=orient(Polygon([bptnp-HD*vec,bptnp+HD*vec,bptnp+HD*(vec+nvec),bptnp+HD*(nvec-vec)]))
        return orient(poly.intersection(rect))
    

def polyxline(poly, bpt, nvec):
    """ (DG)
    Computes the intersection of the boundary of a polygon, poly, and the 
    line defined by a point bpt on the line and its normal nvec.      

    Input:
        poly    : A shapely.geometry.Polygon object 
        bpt     : A basepoint on the line, provided as a shapely.geometry.Point.
        nvec    : NON-ZERO normal vector for the line, provided as a 
                  (1x2 numpy.array)
    Output:
        A list of shapely.geometry.Point objects representing the intersection
        points of the line with the polygon.
    """
    nvec_norm=np.linalg.norm(nvec)
    if (nvec_norm == 0):
        raise ValueError('Normal vector to a line should not vanish')   
    else:
        #form a tangent vector to the line
        bptnp=np.array(bpt.coords[0])
        vec=np.array([-nvec[1],nvec[0]])/nvec_norm

        #to form a line segment through bpt, long enough to contain all intersections, compute
        HD=bpt.hausdorff_distance(poly.boundary)+1
        
        #compute the intersection of the chosen segment with poly.boundary and return it
        seg=LineString([bptnp-HD*vec,bptnp+HD*vec])
        segXpoly=seg.intersection(poly.boundary)
        
        if not(seg.intersects(poly.boundary)):
            # return an empty list if there is no intersection or touching
            return []
        elif isinstance(segXpoly,Point):
            # if the intersection is a single point, return a list with just that point
            return [segXpoly]
        else:
            # in all other cases, concatenate the point lists (intersection might contain segments)
            return np.concatenate([np.array(geom.coords) for geom in segXpoly.geoms])



def polyxray(poly,bpt,vec):
    """ (DG)
    Compute the intersection of the boundary of a polygon,
    with vertex coordinates x and y, and a ray, defined by
    a base point b on the line and a direction vector v.

    Input:
        poly    : A shapely.geometry.Polygon object 
        bpt     : Ray basepoint, provided as a shapely.geometry.Point.
        vec     : NON-ZERO direction vector for the ray, provided as a 
                  (1x2 numpy.array)

    Output:
        The nearest point of intersection between the ray defined by bpt,vec
        and the boundary of the polygon poly
    """
    bptnp=np.array(bpt.coords[0]) #numpy version of the base point
    HD=bpt.hausdorff_distance(poly.boundary)+1 #hausdorff distance of basepoint to polygon boundary, plus 1
    try:
        vec_normalized=vec/np.linalg.norm(vec) #normalized direction vector
    except:
        vec_normalized=np.zeros((1,2)) #no direction if vec is zero

    ray=LineString([bptnp,bptnp+HD*vec_normalized])
    rayXpoly=ray.intersection(poly.boundary)
    if not(ray.intersects(poly.boundary) or ray.touches(poly.boundary)):
        return None
    elif isinstance(rayXpoly,Point):
        return rayXpoly
    else:
        rayXpoly_pts=np.concatenate([np.array(geom.coords) for geom in rayXpoly.geoms])
        proj=lambda x: np.inner(x-bptnp,vec)
        return Point(rayXpoly_pts[np.argmin(proj(rayXpoly_pts))])

def cvxpolyintersect(poly1,poly2):
    """ (DG)
    Compute the intersection of two shapely.Polygon objects

    Input:
        poly1,poly2 : CONVEX shapely.geometry.Polygon objects
    Output:
        The intersection of the input polygons (a Polygon, 2pt-LineString, Point or empty Polygon)
        ***NOTE: THE POINT HERE IS WE RELY ON THE FACT THAT THE INPUTS ARE CONVEX. OTHERWISE, THE OUTPUT
        WILL BE A shapely MULTI-GEOMETRY, RATHER THAN A SINGLE shapely.geometry object of the
        listed type.
    """
    return poly1.intersection(poly2)


def polyerode(poly,offset):
    """ (DG)
    Erosion (Contraction) of a convex polygon, with vertices xy, by a closed 
    circle of radius r

    Input: 
        poly    : a convex polygon provided as shapely.geometry.Polygon
        offset  : Radius of the erosion (contraction) disk
    Output:
        Eroded convex polygon as a shapely.geometry.Polygon
    """
    return poly.buffer(-1.0*offset)


def polydilate(poly,offset,js=2):
    """ (DG)
    Compute the dilation of a polygon by a fixed offset.

    Input: 
        poly    : a convex polygon provided as shapely.geometry.Polygon
        offset  : Radius of the dilation disk
        js      : join style to pass to join_Style kwarg
    Output:
        Dilated convex polygon as a shapely.geometry.Polygon
    """
    return orient(poly.buffer(offset,join_style=js))


def polytriangulation(lring,workspace,touching_boundary):
    """
    Compute the triangulation of the input polygon and its dual (adjacency) graph.

    Input:
        lring               : shapely.LinearRing object (must be valid)
        workspace           : shapely.LinearRing object
        touching_boundary   : Flag that is True if the polygon is touching the boundary of the workspace and False otherwise
    Output:
        tree    : Array of dictionaries with triangles and generated adjacency graph
                  Each dictionary contains:
                    1) 'vertices': vertices of each triangle (arranged CCW - start and end vertices are NOT the same) - for each of the children, the vertices of the adjacency edge are the first two vertices
                    2) 'predecessor': the index of the triangle predecessor in the adjacency tree (-1 for the root)
                    3) 'depth': the depth of the (triangle) node in the adjacency tree (0 for the root)
                    4) 'index': the index of the triangle in the tree (its serial number)
                    5) 'adj_edge': the edge the triangle shares with its predecessor (CCW oriented with respect to the triangle)
    """
    # Construct a CCW simply-connected polygon based on the input coordinate vertices 
    lring = orient(Polygon(lring)).exterior

    # Find polygon vertices (except the last one because of tripy)
    polygon_vertices = np.vstack((lring.xy[0][0:-1], lring.xy[1][0:-1])).transpose()

    # Find triangulation
    triangles = np.array(tripy.earclip(polygon_vertices))

    # Sorting argument for triangles - Area if not touching boundary, Min distance to boundary if touching boundary
    if not touching_boundary:
        sorting_function=lambda tri: Polygon(tri).area
    elif touching_boundary:
        sorting_function=lambda tri: np.min(polydist(Polygon(tri),workspace.coords[0:-1])[0])
    triangles=sorted(triangles,key=sorting_function)    # Sort the triangles

    tree = [dict() for x in range(len(triangles))] #prepare an "empty" output tree
    tree_index = 0 #index of current vertex under construction
    # Construct the first node of the tree that will act as the root
    tree[0]['vertices'] = triangles.pop(0)
    tree[0]['predecessor'] = -1
    tree[0]['depth'] = 0
    tree[0]['index'] = 0
    tree[0]['adj_edge'] = np.array([]) #the root has no predecessor with which to share an edge...

    # Initialize search
    stack = deque([tree[0]])

    # Build the tree by expanding nodes until the stack is empty
    # (The stack will be empty when the leaf nodes consist of only one edge)
    while len(stack)!=0:
        expanded_node = stack.popleft()
        i = 0
        while i<len(triangles):
            # Construct two edge arrays: one for the parent and one for the candidate child
            # Orient the parent CCW as desired and the child CW to check for collisions
            polygon1_edges = np.array([np.vstack((expanded_node['vertices'][0], expanded_node['vertices'][1])), np.vstack((expanded_node['vertices'][1], expanded_node['vertices'][2])), np.vstack((expanded_node['vertices'][2], expanded_node['vertices'][0]))])
            polygon2_edges = np.array([np.vstack((triangles[i][0],triangles[i][2])), np.vstack((triangles[i][2],triangles[i][1])), np.vstack((triangles[i][1], triangles[i][0]))])
            triangles_touch = False
            for polygon1_edge_index in range(polygon1_edges.shape[0]):
                for polygon2_edge_index in range(polygon2_edges.shape[0]):
                    if (np.abs(polygon1_edges[polygon1_edge_index]-polygon2_edges[polygon2_edge_index])<1e-5).all():
                        triangles_touch=True
                        adj_edge_index=polygon2_edge_index

            # Check if the triangles touch, otherwise continue
            if not triangles_touch:
                i = i+1
                continue
            else:
                # Add the child to the tree
                tree_index = tree_index+1
                tree[tree_index]['predecessor'] = expanded_node['index']
                tree[tree_index]['depth'] = tree[tree[tree_index]['predecessor']]['depth']+1
                tree[tree_index]['index'] = tree_index
                tree[tree_index]['adj_edge'] = polygon2_edges[adj_edge_index]

                # Find the 3rd point of the child triangle (that does not belong to the shared edge) and arrange the vertices so that this is the 3rd point
                nrows, ncols = triangles[i].shape
                dtype = {'names':['f{}'.format(j) for j in range(ncols)], 'formats':ncols * [triangles[i].dtype]} 
                set_diff = np.setdiff1d(triangles[i].view(dtype), np.ascontiguousarray(tree[tree_index]['adj_edge'].transpose()).view(dtype))
                third_vertex = set_diff.view(triangles[i].dtype).reshape(-1, ncols)
                tree[tree_index]['vertices'] = np.array([tree[tree_index]['adj_edge'][1], tree[tree_index]['adj_edge'][0], third_vertex[0]]) # change the direction of adj_edge to make the child CCW again 

                # Delete the child from the input
                triangles.pop(i)

                # Add the child to the stack to be expanded
                stack.append(tree[tree_index])

    # As a final step, sort the tree as a stack, in order of descending depth
    tree = sorted(tree, key=itemgetter('depth'), reverse=True)

    # Make sure to change the node and predecessor indices since the indices have now changed
    indices_new = np.zeros(len(tree))
    indices_old = np.zeros(len(tree))
    for i in range(0,len(tree)):
        indices_new[i] = i
        indices_old[i] = tree[i]['index']

    for i in range(0,len(tree)-1):
        tree[i]['predecessor'] = int(indices_new[indices_old==tree[i]['predecessor']][0])
        tree[i]['index'] = i
    tree[len(tree)-1]['index'] = len(tree)-1
    
    return tree

def polyconvexdecomposition(lring,workspace,touching_boundary):
    """ (DG)
    Compute the convex decomposition of the input polygon and its dual (adjacency) graph.

    Input:
        lring               : shapely.LinearRing object (must be valid)
        workspace           : shapely.LinearRing object
        touching_boundary   : Flag that is True if the polygon is touching the boundary of the workspace and False otherwise
    Output:
        tree    : Array of dictionaries with polygons and generated adjacency graph
                  Each dictionary contains:
                    1) 'vertices': vertices of each polygon (arranged CCW - start and end vertices are NOT the same) - for each of the children, the vertices of the adjacency edge are the first two vertices
                    2) 'predecessor': the index of the polygon predecessor in the adjacency tree (-1 for the root)
                    3) 'depth': the depth of the (polygon) node in the adjacency tree (0 for the root)
                    4) 'index': the index of the polygon in the tree (its serial number)
                    5) 'adj_edge': the edge the polygon shares with its predecessor (CCW oriented with respect to the polygon)
    """
    # Construct a CCW simply-connected polygon based on the input coordinate vertices 
    lring = orient(Polygon(lring)).exterior

    # Reformat polygon vertices for polycvxdecomp
    polygon_vertices = np.vstack((lring.xy[0][0:-1], lring.xy[1][0:-1])).transpose()

    #DG
    # Find convex decomposition and remove parts with small area
    polygons=filter(lambda pol: Polygon(pol).area>=0.01,polycvxdecomp(polygon_vertices.tolist()))
    polygons=[np.array(xi) for xi in polygons] #turn into numpy arrays

    # Sorting argument for polygons - Area if not touching boundary, Min distance to boundary if touching boundary
    if not touching_boundary:
        sorting_function=lambda pol: -Polygon(pol).area
    elif touching_boundary:
        sorting_function=lambda pol: float(not(Polygon(workspace).exterior.intersection(Polygon(pol)).geom_type == 'LineString'))
        #sorting_function=lambda pol: np.min(polydist(Polygon(pol),workspace.coords[0:-1])[0])
    
    # Sort the polygons
    polygons=sorted(polygons,key=sorting_function)

    # Construct the first node of the tree that will act as the root
    tree = [dict() for x in range(len(polygons))]
    tree[0]['vertices'] = polygons.pop(0)
    tree[0]['predecessor'] = -1
    tree[0]['depth'] = 0
    tree[0]['index'] = 0
    tree[0]['adj_edge'] = np.array([])
    tree_index = 0

    # Initialize search
    stack = deque([tree[0]])

    # Build the tree by expanding nodes until the stack is empty
    while len(stack)!=0:
        # Pop the first element of the stack and delete it from the stack
        expanded_node = stack.popleft()

        # Find edges of expanded node - CW
        polygon1_edges = []
        for j in range(0,expanded_node['vertices'].shape[0]):
            polygon1_edges.append(np.array([expanded_node['vertices'][(j+1)%expanded_node['vertices'].shape[0]], expanded_node['vertices'][j%expanded_node['vertices'].shape[0]]]))
        polygon1_edges = np.array(polygon1_edges)

        i = 0
        while i<len(polygons):
            # Find edges of candidate child - CCW
            polygon2_edges = []
            for j in range(0,polygons[i].shape[0]):
                polygon2_edges.append(np.array([polygons[i][j%polygons[i].shape[0]], polygons[i][(j+1)%polygons[i].shape[0]]]))
            polygon2_edges = np.array(polygon2_edges)

            polygons_touch = False
            for polygon1_edge_index in range(polygon1_edges.shape[0]):
                for polygon2_edge_index in range(polygon2_edges.shape[0]):
                    if (np.abs(polygon1_edges[polygon1_edge_index]-polygon2_edges[polygon2_edge_index])<1e-5).all():
                        polygons_touch = True
                        adj_edge_index = polygon2_edge_index

            # Check if the polygons touch, otherwise continue
            if not polygons_touch:
                i = i+1
                continue
            else:
                # Add the child to the tree with the adjacency edge being first
                tree_index = tree_index+1
                tree[tree_index]['predecessor'] = expanded_node['index']
                tree[tree_index]['depth'] = tree[tree[tree_index]['predecessor']]['depth']+1
                tree[tree_index]['index'] = tree_index
                tree[tree_index]['adj_edge'] = polygon2_edges[adj_edge_index]
                tree[tree_index]['vertices'] = np.roll(polygons[i],-adj_edge_index,axis=0)

                # As a final preprocessing step, check whether the edges before and after the adj_edge are parallel with adj_edge
                # If that's the case, cut the triangle corresponding to that edge as an extra polygon
                tangent_before = np.array(tree[tree_index]['vertices'][0]-tree[tree_index]['vertices'][-1])/np.linalg.norm(tree[tree_index]['vertices'][0]-tree[tree_index]['vertices'][-1])
                tangent_after = np.array(tree[tree_index]['vertices'][2]-tree[tree_index]['vertices'][1])/np.linalg.norm(tree[tree_index]['vertices'][2]-tree[tree_index]['vertices'][1])
                tangent_adj_edge = np.array(tree[tree_index]['adj_edge'][1]-tree[tree_index]['adj_edge'][0])/np.linalg.norm(tree[tree_index]['adj_edge'][1]-tree[tree_index]['adj_edge'][0])
                normal_adj_edge = np.array([-tangent_adj_edge[1],tangent_adj_edge[0]])
                if np.abs(np.dot(tangent_before,normal_adj_edge)) < 0.001:
                    # Add triangle
                    tree_before = dict()
                    tree_before['predecessor'] = tree_index
                    tree_before['depth'] = tree[tree_index]['depth']+1
                    tree_before['index'] = len(tree)
                    tree_before['adj_edge'] = np.vstack((tree[tree_index]['vertices'][0],tree[tree_index]['vertices'][-2]))
                    tree_before['vertices'] = np.vstack((tree[tree_index]['vertices'][0],tree[tree_index]['vertices'][-2],tree[tree_index]['vertices'][-1]))

                    # Delete the last vertex from the original polygon
                    tree[tree_index]['vertices'] = np.delete(tree[tree_index]['vertices'], -1, axis=0)

                    # Add the new triangle to the tree and stack
                    tree.append(tree_before)
                    stack.append(tree_before)
                
                if np.abs(np.dot(tangent_after,normal_adj_edge)) < 0.001:
                    # Add triangle
                    tree_after = dict()
                    tree_after['predecessor'] = tree_index
                    tree_after['depth'] = tree[tree_index]['depth']+1
                    tree_after['index'] = len(tree)
                    tree_after['adj_edge'] = np.vstack((tree[tree_index]['vertices'][3],tree[tree_index]['vertices'][1]))
                    tree_after['vertices'] = np.vstack((tree[tree_index]['vertices'][3],tree[tree_index]['vertices'][1],tree[tree_index]['vertices'][2]))

                    # Delete the third vertex from the original polygon
                    tree[tree_index]['vertices'] = np.delete(tree[tree_index]['vertices'], 2, axis=0)

                    # Add the new triangle to the tree and stack
                    tree.append(tree_after)
                    stack.append(tree_after)

                # Delete the child from the input
                polygons.pop(i)

                # Add the child to the stack to be expanded
                stack.append(tree[tree_index])

    # As a final step, sort the tree as a stack, in order of descending depth
    tree = sorted(tree, key=itemgetter('depth'), reverse=True)

    # Make sure to change the node and predecessor indices since the indices have now changed
    indices_new = np.zeros(len(tree))
    indices_old = np.zeros(len(tree))
    for i in range(0,len(tree)):
        indices_new[i] = i
        indices_old[i] = tree[i]['index']

    for i in range(0,len(tree)-1):
        tree[i]['predecessor'] = int(indices_new[indices_old==tree[i]['predecessor']][0])
        tree[i]['index'] = i
    tree[len(tree)-1]['index'] = len(tree)-1
    
    return tree

def polyintersect(xy1,xy2):
    """
    Checks if polygon xy1 intersects polygon xy2

    Input:
        xy1     : Vertex Coordinates of a polygon
                  (Nx2 numpy.array)
        xy2     : Vertex Coordinates of a polygon
                  (Nx2 numpy.array)
    Output:
        outcome : True if xy1 intersects xy2, False otherwise
    """

    # Construct polygon objects based on the input vertices
    polygon1 = Polygon(xy1)
    polygon2 = Polygon(xy2)

    outcome = polygon1.intersects(polygon2)

    return outcome

def polyunion(poly1,poly2):
    """ (DG)
    Computes the union of two polygons xy1 and xy2

    Input:
        poly1,poly2 : shapely.geometry.Polygon polygons
    Output:
        The union of the two overlapping polygons, provided as a shapely.geometry.Polygon
    """
    return orient(unary_union([poly1,poly2])).simplify(0.01)

def lineint(l1, l2, precision=0):
    """Compute the intersection between two lines.

    Input:
        l1 : first line
        l2 : second line
        precision : precision to check if lines are parallel (default 0)

    Output:
        The intersection point
    """
    i = [0, 0] # point
    a1 = l1[1][1] - l1[0][1]
    b1 = l1[0][0] - l1[1][0]
    c1 = a1 * l1[0][0] + b1 * l1[0][1]
    a2 = l2[1][1] - l2[0][1]
    b2 = l2[0][0] - l2[1][0]
    c2 = a2 * l2[0][0] + b2 * l2[0][1]
    det = a1 * b2 - a2 * b1
    if not scalar_eq(det, 0, precision): # lines are not parallel
        i[0] = (b2 * c1 - b1 * c2) / det
        i[1] = (a1 * c2 - a2 * c1) / det
    return i

def linesegmentsintersect(p1, p2, q1, q2):
    """Checks if two line segments intersect.

    Input:
        p1 : The start vertex of the first line segment.
        p2 : The end vertex of the first line segment.
        q1 : The start vertex of the second line segment.
        q2 : The end vertex of the second line segment.

    Output:
        True if the two line segments intersect
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    da = q2[0] - q1[0]
    db = q2[1] - q1[1]

    # segments are parallel
    if (da*dy - db*dx) == 0:
        return False

    s = (dx * (q1[1] - p1[1]) + dy * (p1[0] - q1[0])) / (da * dy - db * dx)
    t = (da * (p1[1] - q1[1]) + db * (q1[0] - p1[0])) / (db * dx - da * dy)

    return s >= 0 and s <= 1 and t >= 0 and t <= 1

def trianglearea(a, b, c):
    """Calculates the area of a triangle spanned by three points.
    Note that the area will be negative if the points are not given in counter-clockwise order.

    Input:
        a : First point
        b : Second point
        c : Third point

    Output:
        Area of triangle
    """
    return ((b[0] - a[0])*(c[1] - a[1]))-((c[0] - a[0])*(b[1] - a[1]))

def isleft(a, b, c):
    return trianglearea(a, b, c) > 0

def islefton(a, b, c):
    return trianglearea(a, b, c) >= 0

def isright(a, b, c):
    return trianglearea(a, b, c) < 0

def isrighton(a, b, c):
    return trianglearea(a, b, c) <= 0

def collinear(a, b, c, thresholdAngle=0):
    """Checks if three points are collinear.

    Input:
        a : First point
        b : Second point
        c : Third point
        thresholdAngle : threshold to consider if points are collinear, in radians (default 0)

    Output:
        True if points are collinear
    """
    if thresholdAngle == 0:
        return trianglearea(a, b, c) == 0
    else:
        ab = [None] * 2
        bc = [None] * 2

        ab[0] = b[0]-a[0]
        ab[1] = b[1]-a[1]
        bc[0] = c[0]-b[0]
        bc[1] = c[1]-b[1]

        dot = ab[0]*bc[0] + ab[1]*bc[1]
        magA = math.sqrt(ab[0]*ab[0] + ab[1]*ab[1])
        magB = math.sqrt(bc[0]*bc[0] + bc[1]*bc[1])
        angle = math.acos(dot/(magA*magB))
        return angle < thresholdAngle

def sqdist(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return dx * dx + dy * dy

def polyat(polygon, i):
    """Gets a vertex at position i on the polygon.
    It does not matter if i is out of bounds.

    Input:
        polygon : The polygon
        i : Position desired on the polygon

    Output:
        Vertex at position i
    """
    s = len(polygon)
    return polygon[i % s]

def polyclear(polygon):
    """Clears the polygon data

    Input:
        polygon : The polygon
    """
    del polygon[:]

def polyappend(polygon, poly, start, end):
    """Grabs points at indicies `start` to `end` from `poly`
    and appends them to `polygon`

    Input:
        polygon : The destination polygon
        poly : The source polygon
        start : Starting source index
        end : Ending source index (not included in the slice)
    """
    for i in range(start, end):
        polygon.append(poly[i])

def polymakeccw(polygon):
    """Makes sure that the polygon vertices are ordered counter-clockwise.

    Input:
        polygon : The polygon
    """
    br = 0
    v = polygon

    # find bottom right point
    for i in range(1, len(polygon)):
        if v[i][1] < v[br][1] or (v[i][1] == v[br][1] and v[i][0] > v[br][0]):
            br = i

    # reverse poly if clockwise
    if not isleft(polyat(polygon, br - 1), polyat(polygon, br), polyat(polygon, br + 1)):
        polyreverse(polygon)

def polyreverse(polygon):
    """Reverses the vertices in the polygon.

    Input:
        polygon : The polygon
    """
    polygon.reverse()

def polyisreflex(polygon, i):
    """Checks if a point in the polygon is a reflex point.

    Input:
        polygon : The polygon
        i : index of point to check
    
    Output:
        True iff point is a reflex point
    """
    return isright(polyat(polygon, i - 1), polyat(polygon, i), polyat(polygon, i + 1))

def polycansee(polygon, a, b):
    """Checks if two vertices in the polygon can see each other.

    Input:
        polygon : The polygon
        a : Vertex 1
        b : Vertex 2

    Output:
        True if vertices can see each other
    """

    l1 = [None]*2
    l2 = [None]*2

    if islefton(polyat(polygon, a + 1), polyat(polygon, a), polyat(polygon, b)) and isrighton(polyat(polygon, a - 1), polyat(polygon, a), polyat(polygon, b)):
        return False

    dist = sqdist(polyat(polygon, a), polyat(polygon, b))
    for i in range(0, len(polygon)): # for each edge
        if (i + 1) % len(polygon) == a or i == a: # ignore incident edges
            continue

        if islefton(polyat(polygon, a), polyat(polygon, b), polyat(polygon, i + 1)) and isrighton(polyat(polygon, a), polyat(polygon, b), polyat(polygon, i)): # if diag intersects an edge
            l1[0] = polyat(polygon, a)
            l1[1] = polyat(polygon, b)
            l2[0] = polyat(polygon, i)
            l2[1] = polyat(polygon, i + 1)
            p = lineint(l1, l2)
            if sqdist(polyat(polygon, a), p) < dist: # if edge is blocking visibility to b
                return False

    return True

def polycopy(polygon, i, j, targetPoly=None):
    """Copies the polygon from vertex i to vertex j to targetPoly.

    Input:
        polygon : The source polygon
        i : start vertex
        j : end vertex (inclusive)
        targetPoly -- Optional target polygon

    Output:
        The resulting copy.
    """
    p = targetPoly or []
    polyclear(p)
    if i < j:
        # Insert all vertices from i to j
        for k in range(i, j+1):
            p.append(polygon[k])

    else:
        # Insert vertices 0 to j
        for k in range(0, j+1):
            p.append(polygon[k])

        # Insert vertices i to end
        for k in range(i, len(polygon)):
            p.append(polygon[k])

    return p

def polygetcutedges(polygon):
    """Decomposes the polygon into convex pieces.
    Note that this algorithm has complexity O(N^4) and will be very slow for polygons with many vertices.

    Input:
        polygon : The polygon

    Output:
        A list of edges [[p1,p2],[p2,p3],...] that cut the polygon.
    """
    mins = []
    tmp1 = []
    tmp2 = []
    tmpPoly = []
    nDiags = float('inf')

    for i in range(0, len(polygon)):
        if polyisreflex(polygon, i):
            for j in range(0, len(polygon)):
                if polycansee(polygon, i, j):
                    tmp1 = polygetcutedges(polycopy(polygon, i, j, tmpPoly))
                    tmp2 = polygetcutedges(polycopy(polygon, j, i, tmpPoly))

                    for k in range(0, len(tmp2)):
                        tmp1.append(tmp2[k])

                    if len(tmp1) < nDiags:
                        mins = tmp1
                        nDiags = len(tmp1)
                        mins.append([polyat(polygon, i), polyat(polygon, j)])

    return mins

def polydecomp(polygon):
    """Decomposes the polygon into one or more convex sub-polygons.

    Input:
        polygon : The polygon

    Output:
        An array or polygon objects.
    """
    edges = polygetcutedges(polygon)
    if len(edges) > 0:
        return polyslice(polygon, edges)
    else:
        return [polygon]

def polyslice(polygon, cutEdges):
    """Slices the polygon given one or more cut edges. If given one, this function will return two polygons (false on failure). If many, an array of polygons.
    
    Input:
        polygon : The polygon
        cutEdges : A list of edges to cut on, as returned by getCutEdges()
    
    Output:
        An array of polygon objects.
    """
    if len(cutEdges) == 0:
        return [polygon]

    if isinstance(cutEdges, list) and len(cutEdges) != 0 and isinstance(cutEdges[0], list) and len(cutEdges[0]) == 2 and isinstance(cutEdges[0][0], list):

        polys = [polygon]

        for i in range(0, len(cutEdges)):
            cutEdge = cutEdges[i]
            # Cut all polys
            for j in range(0, len(polys)):
                poly = polys[j]
                result = polyslice(poly, cutEdge)
                if result:
                    # Found poly! Cut and quit
                    del polys[j:j+1]
                    polys.extend((result[0], result[1]))
                    break

        return polys
    else:

        # Was given one edge
        cutEdge = cutEdges
        i = polygon.index(cutEdge[0])
        j = polygon.index(cutEdge[1])

        if i != -1 and j != -1:
            return [polycopy(polygon, i, j),
                    polycopy(polygon, j, i)]
        else:
            return False

def polyissimple(polygon):
    """Checks that the line segments of this polygon do not intersect each other.
    
    Input:
        polygon : The polygon
    
    Output:
        True is polygon is simple (not self-intersecting)
    
    Todo:
        Should it check all segments with all others?
    """
    path = polygon
    # Check
    for i in range(0,len(path)-1):
        for j in range(0, i-1):
            if linesegmentsintersect(path[i], path[i+1], path[j], path[j+1]):
                return False

    # Check the segment between the last and the first point to all others
    for i in range(1,len(path)-2):
        if linesegmentsintersect(path[0], path[len(path)-1], path[i], path[i+1]):
            return False

    return True

def getintersection(p1, p2, q1, q2, delta=0):
    """Gets the intersection point 
    
    Input:
        p1 : The start vertex of the first line segment.
        p2 : The end vertex of the first line segment.
        q1 : The start vertex of the second line segment.
        q2 : The end vertex of the second line segment.
        delta : Optional precision to check if lines are parallel (default 0)
    
    Output:
        The intersection point.
    """
    a1 = p2[1] - p1[1]
    b1 = p1[0] - p2[0]
    c1 = (a1 * p1[0]) + (b1 * p1[1])
    a2 = q2[1] - q1[1]
    b2 = q1[0] - q2[0]
    c2 = (a2 * q1[0]) + (b2 * q1[1])
    det = (a1 * b2) - (a2 * b1)

    if not scalar_eq(det, 0, delta):
        return [((b2 * c1) - (b1 * c2)) / det, ((a1 * c2) - (a2 * c1)) / det]
    else:
        return [0, 0]

def polycvxdecomp(polygon, result=None, reflexVertices=None, steinerPoints=None, delta=25, maxlevel=10000, level=0):
    """Quickly decompose the Polygon into convex sub-polygons. Algorithm based on Mark Bayazit's polygon decomposition.
    
    Input:
        polygon : The polygon to decompose
        result : Stores result of decomposed polygon, passed recursively
        reflexVertices : 
        steinerPoints :
        delta : Currently unused
        maxlevel : The maximum allowed level of recursion
        level : The current level of recursion
    
    Output:
        List of decomposed convex polygons
    """
    if result is None:
        result = []
    reflexVertices = reflexVertices or []
    steinerPoints = steinerPoints or []

    upperInt = [0, 0]
    lowerInt = [0, 0]
    p = [0, 0]         # Points
    upperDist = 0
    lowerDist = 0
    d = 0
    closestDist = 0 # scalars
    upperIndex = 0
    lowerIndex = 0
    closestIndex = 0 # integers
    lowerPoly = []
    upperPoly = [] # polygons
    poly = polygon
    v = polygon

    if len(v) < 3:
        return result

    level += 1
    if level > maxlevel:
        print("quickDecomp: max level ("+str(maxlevel)+") reached.")
        return result

    for i in range(0, len(polygon)):
        if polyisreflex(poly, i):
            reflexVertices.append(poly[i])
            upperDist = float('inf')
            lowerDist = float('inf')

            for j in range(0, len(polygon)):
                if isleft(polyat(poly, i - 1), polyat(poly, i), polyat(poly, j)) and isrighton(polyat(poly, i - 1), polyat(poly, i), polyat(poly, j - 1)): # if line intersects with an edge
                    p = getintersection(polyat(poly, i - 1), polyat(poly, i), polyat(poly, j), polyat(poly, j - 1)) # find the point of intersection
                    if isright(polyat(poly, i + 1), polyat(poly, i), p): # make sure it's inside the poly
                        d = sqdist(poly[i], p)
                        if d < lowerDist: # keep only the closest intersection
                            lowerDist = d
                            lowerInt = p
                            lowerIndex = j

                if isleft(polyat(poly, i + 1), polyat(poly, i), polyat(poly, j + 1)) and isrighton(polyat(poly, i + 1), polyat(poly, i), polyat(poly, j)):
                    p = getintersection(polyat(poly, i + 1), polyat(poly, i), polyat(poly, j), polyat(poly, j + 1))
                    if isleft(polyat(poly, i - 1), polyat(poly, i), p):
                        d = sqdist(poly[i], p)
                        if d < upperDist:
                            upperDist = d
                            upperInt = p
                            upperIndex = j

            # if there are no vertices to connect to, choose a point in the middle
            if lowerIndex == (upperIndex + 1) % len(polygon):
                #print("Case 1: Vertex("+str(i)+"), lowerIndex("+str(lowerIndex)+"), upperIndex("+str(upperIndex)+"), poly.size("+str(len(polygon))+")")
                p[0] = (lowerInt[0] + upperInt[0]) / 2
                p[1] = (lowerInt[1] + upperInt[1]) / 2
                steinerPoints.append(p)

                if i < upperIndex:
                    #lowerPoly.insert(lowerPoly.end(), poly.begin() + i, poly.begin() + upperIndex + 1)
                    polyappend(lowerPoly, poly, i, upperIndex+1)
                    lowerPoly.append(p)
                    upperPoly.append(p)
                    if lowerIndex != 0:
                        #upperPoly.insert(upperPoly.end(), poly.begin() + lowerIndex, poly.end())
                        polyappend(upperPoly, poly, lowerIndex, len(poly))

                    #upperPoly.insert(upperPoly.end(), poly.begin(), poly.begin() + i + 1)
                    polyappend(upperPoly, poly, 0, i+1)
                else:
                    if i != 0:
                        #lowerPoly.insert(lowerPoly.end(), poly.begin() + i, poly.end())
                        polyappend(lowerPoly, poly, i, len(poly))

                    #lowerPoly.insert(lowerPoly.end(), poly.begin(), poly.begin() + upperIndex + 1)
                    polyappend(lowerPoly, poly, 0, upperIndex+1)
                    lowerPoly.append(p)
                    upperPoly.append(p)
                    #upperPoly.insert(upperPoly.end(), poly.begin() + lowerIndex, poly.begin() + i + 1)
                    polyappend(upperPoly, poly, lowerIndex, i+1)

            else:
                # connect to the closest point within the triangle
                #print("Case 2: Vertex("+str(i)+"), closestIndex("+str(closestIndex)+"), poly.size("+str(len(polygon))+")\n")

                if lowerIndex > upperIndex:
                    upperIndex += len(polygon)

                closestDist = float('inf')

                if upperIndex < lowerIndex:
                    return result

                for j in range(lowerIndex, upperIndex+1):
                    if islefton(polyat(poly, i - 1), polyat(poly, i), polyat(poly, j)) and isrighton(polyat(poly, i + 1), polyat(poly, i), polyat(poly, j)):
                        d = sqdist(polyat(poly, i), polyat(poly, j))
                        if d < closestDist:
                            closestDist = d
                            closestIndex = j % len(polygon)

                if i < closestIndex:
                    polyappend(lowerPoly, poly, i, closestIndex+1)
                    if closestIndex != 0:
                        polyappend(upperPoly, poly, closestIndex, len(v))

                    polyappend(upperPoly, poly, 0, i+1)
                else:
                    if i != 0:
                        polyappend(lowerPoly, poly, i, len(v))

                    polyappend(lowerPoly, poly, 0, closestIndex+1)
                    polyappend(upperPoly, poly, closestIndex, i+1)

            # solve smallest poly first
            if len(lowerPoly) < len(upperPoly):
                polycvxdecomp(lowerPoly, result, reflexVertices, steinerPoints, delta, maxlevel, level)
                polycvxdecomp(upperPoly, result, reflexVertices, steinerPoints, delta, maxlevel, level)
            else:
                polycvxdecomp(upperPoly, result, reflexVertices, steinerPoints, delta, maxlevel, level)
                polycvxdecomp(lowerPoly, result, reflexVertices, steinerPoints, delta, maxlevel, level)

            return result

    result.append(polygon)

    return result

def polyremovecollinear(polygon, precision=0):
    """Remove collinear points in the polygon.
    
    Input:
        polygon : The polygon
        precision : The threshold angle to use when determining whether two edges are collinear. (default is 0)
    
    Output:
        The number of points removed
    """
    num = 0
    i = len(polygon) - 1
    while len(polygon) > 3 and i >= 0:
    #(var i=polygon.length-1; polygon.length>3 && i>=0; --i){
        if collinear(polyat(polygon, i - 1), polyat(polygon, i), polyat(polygon, i+1), precision):
            # Remove the middle point
            del polygon[i % len(polygon):(i % len(polygon))+1]
            num += 1
        i -= 1
    return num

def scalar_eq(a, b, precision=0):
    """Check if two scalars are equal.
    
    Input:
        a : first scalar
        b : second scalar
        precision : precision to check equality
    
    Output:
        True if scalars are equal to the specified precision
    """
    return abs(a - b) <= precision