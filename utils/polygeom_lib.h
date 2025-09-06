#ifndef POLYGEOM_LIB_H
#define POLYGEOM_LIB_H

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <iterator>

// Boost Geometry includes
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
namespace bg = boost::geometry;

// CGAL includes (for convex decomposition)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>

// Mapbox earcut (for triangulation)
#include <mapbox/earcut.hpp>

// Type definitions for Boost Geometry
typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::polygon<point> polygon;
typedef bg::model::linestring<point> line;

// Type definitions for CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 CGAL_Point_2;
typedef CGAL::Polygon_2<K> CGAL_Polygon_2;
typedef std::list<CGAL_Polygon_2> CGAL_Polygon_list;
typedef CGAL::Partition_traits_2<K> CGAL_Traits;

// Custom structs for return values
struct ProjectionResultStruct {
    double dist;
    point projected_point;
};

// Class definitions for triangle and polygon trees
class TriangleClass {
private:
    std::vector<point> vertices_;
    int predecessor_;
    int depth_;
    int index_;
    std::vector<point> adj_edge_;

public:
    // Getters
    std::vector<point> get_vertices() const { return vertices_; }
    int get_predecessor() const { return predecessor_; }
    int get_depth() const { return depth_; }
    int get_index() const { return index_; }
    std::vector<point> get_adj_edge() const { return adj_edge_; }

    // Setters
    void set_vertices(const std::vector<point>& vertices) { vertices_ = vertices; }
    void set_predecessor(int predecessor) { predecessor_ = predecessor; }
    void set_depth(int depth) { depth_ = depth; }
    void set_index(int index) { index_ = index; }
    void set_adj_edge(const std::vector<point>& adj_edge) { adj_edge_ = adj_edge; }
};

class PolygonClass {
private:
    std::vector<point> vertices_;
    int predecessor_;
    int depth_;
    int index_;
    std::vector<point> adj_edge_;

public:
    // Getters
    std::vector<point> get_vertices() const { return vertices_; }
    int get_predecessor() const { return predecessor_; }
    int get_depth() const { return depth_; }
    int get_index() const { return index_; }
    std::vector<point> get_adj_edge() const { return adj_edge_; }

    // Setters
    void set_vertices(const std::vector<point>& vertices) { vertices_ = vertices; }
    void set_predecessor(int predecessor) { predecessor_ = predecessor; }
    void set_depth(int depth) { depth_ = depth; }
    void set_index(int index) { index_ = index; }
    void set_adj_edge(const std::vector<point>& adj_edge) { adj_edge_ = adj_edge; }
};

// Function declarations

// Basic linear algebra operations
std::vector<std::vector<double>> MatrixMatrixMultiplication(std::vector<std::vector<double>> Matrix1, std::vector<std::vector<double>> Matrix2);
std::vector<double> MatrixVectorMultiplication(std::vector<std::vector<double>> Matrix, std::vector<double> Vector);
std::vector<std::vector<double>> VectorOuterProduct(std::vector<double> Vector1, std::vector<double> Vector2);
double MatrixDeterminant(std::vector<std::vector<double>> Matrix);

// Data conversion functions
std::vector<point> StdToBoostPoint(std::vector<std::vector<double>> input);
std::vector<std::vector<double>> BoostPointToStd(std::vector<point> input);
std::vector<point> BoostPolyToBoostPoint(polygon input);
std::vector<point> BoostLineToBoostPoint(line input);
polygon BoostPointToBoostPoly(std::vector<point> input);
line BoostPointToBoostLine(std::vector<point> input);

// Utility functions
double angle_transformation(double input_angle);

// Geometric intersection operations
polygon cvxpolyxhplane(polygon xy, point m, point n);
line polyxline(polygon xy, point m, point n);
point polyxray(polygon xy, point b, point v);

// Distance and projection calculations
ProjectionResultStruct polydist(polygon xy, point p);
ProjectionResultStruct linedist(line xy, point p);

// Advanced polygon decomposition
void polytriangulation(std::vector<std::vector<double>> xy, std::vector<std::vector<double>> workspace, bool touching_boundary, std::vector<TriangleClass> *tree);
void polyconvexdecomposition(std::vector<std::vector<double>> xy, std::vector<std::vector<double>> workspace, bool touching_boundary, std::vector<PolygonClass> *tree);

#endif // POLYGEOM_LIB_H
