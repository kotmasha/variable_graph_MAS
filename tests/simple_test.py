#!/usr/bin/env python3
"""
Simple command-line test script for polygeom operations
No matplotlib dependencies - just prints results
"""

import sys
import os
import math
import time

# Add utils directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from polygeom_wrapper import PolyGeom, PolyGeomError

def test_basic_operations():
    """Test basic mathematical operations"""
    print("=" * 60)
    print("TESTING BASIC MATHEMATICAL OPERATIONS")
    print("=" * 60)
    
    pg = PolyGeom()
    
    # Test matrix determinant
    matrices = [
        [[1, 2], [3, 4]],  # Expected: -2
        [[2, 0], [0, 3]],  # Expected: 6
        [[1, 0], [0, 1]],  # Expected: 1 (identity)
        [[1, 1], [1, 1]]   # Expected: 0 (singular)
    ]
    
    print("Matrix Determinant Tests:")
    for i, matrix in enumerate(matrices, 1):
        det = pg.matrix_determinant(matrix)
        expected = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0]
        status = "✓" if abs(det - expected) < 1e-10 else "✗"
        print(f"  Test {i}: {matrix} → det = {det} {status}")
    
    # Test matrix-vector multiplication
    print("\nMatrix-Vector Multiplication Tests:")
    matrix = [[2, 1], [1, 2]]
    vectors = [(1, 0), (0, 1), (1, 1), (2, -1)]
    
    for i, vector in enumerate(vectors, 1):
        result = pg.matrix_vector_multiply(matrix, list(vector))
        expected = [2*vector[0] + 1*vector[1], 1*vector[0] + 2*vector[1]]
        status = "✓" if all(abs(r - e) < 1e-10 for r, e in zip(result, expected)) else "✗"
        print(f"  Test {i}: {matrix} × {vector} → {result} {status}")
    
    # Test angle transformation
    print("\nAngle Transformation Tests:")
    test_angles = [0, math.pi/2, math.pi, 3*math.pi/2, 2*math.pi, -math.pi, -3*math.pi]
    
    for i, angle in enumerate(test_angles, 1):
        normalized = pg.angle_transformation(angle)
        in_range = -math.pi <= normalized <= math.pi
        status = "✓" if in_range else "✗"
        print(f"  Test {i}: {angle:.3f} → {normalized:.3f} ({in_range}) {status}")

def test_geometric_operations():
    """Test geometric operations"""
    print("\n" + "=" * 60)
    print("TESTING GEOMETRIC OPERATIONS")
    print("=" * 60)
    
    pg = PolyGeom()
    
    # Test polygon: unit square
    square = [(0, 0), (4, 0), (4, 4), (0, 4)]
    
    print("Point-to-Polygon Distance Tests (4×4 square):")
    test_points = [
        ((2, 2), 0.0, "Point inside"),         # Inside
        ((2, -1), 1.0, "Point below"),        # Below
        ((6, 2), 2.0, "Point right"),         # Right
        ((-1, 2), 1.0, "Point left"),         # Left
        ((2, 6), 2.0, "Point above"),         # Above
        ((0, 0), 0.0, "Corner point"),        # On corner
        ((2, 0), 0.0, "Edge point")           # On edge
    ]
    
    for i, (point, expected_dist, description) in enumerate(test_points, 1):
        try:
            distance, closest = pg.point_to_polygon_distance(square, point)
            # Tolerance for distance comparison
            dist_ok = abs(distance - expected_dist) < 0.1
            status = "✓" if dist_ok else "✗"
            print(f"  Test {i}: {point} → dist={distance:.2f}, closest={closest} ({description}) {status}")
        except Exception as e:
            print(f"  Test {i}: {point} → ERROR: {e} ✗")
    
    print("\nRay-Polygon Intersection Tests:")
    ray_tests = [
        ((2, 2), (1, 0), "Right from center"),     # Should hit (4, 2)
        ((2, 2), (0, 1), "Up from center"),       # Should hit (2, 4)
        ((2, 2), (-1, 0), "Left from center"),    # Should hit (0, 2)
        ((2, 2), (0, -1), "Down from center"),    # Should hit (2, 0)
        ((6, 2), (-1, 0), "Left from outside"),   # Should hit (4, 2)
        ((2, 6), (0, -1), "Down from outside")    # Should hit (2, 4)
    ]
    
    for i, (origin, direction, description) in enumerate(ray_tests, 1):
        try:
            intersection = pg.polygon_ray_intersection(square, origin, direction)
            status = "✓" if intersection is not None else "?"
            if intersection:
                print(f"  Test {i}: {origin} + t*{direction} → {intersection} ({description}) {status}")
            else:
                print(f"  Test {i}: {origin} + t*{direction} → No intersection ({description}) {status}")
        except Exception as e:
            print(f"  Test {i}: {origin} + t*{direction} → ERROR: {e} ✗")

def test_polygon_operations():
    """Test polygon-specific operations"""
    print("\n" + "=" * 60)
    print("TESTING POLYGON OPERATIONS")
    print("=" * 60)
    
    pg = PolyGeom()
    
    # Test triangulation
    print("Triangulation Tests:")
    test_polygons = [
        ([(0, 0), (4, 0), (4, 4), (0, 4)], "Square"),
        ([(1, 1), (3, 1), (4, 3), (2, 4), (0, 3)], "Pentagon"),
        ([(0, 0), (3, 0), (4, 2), (3, 4), (0, 4), (-1, 2)], "Hexagon")
    ]
    
    for i, (polygon, description) in enumerate(test_polygons, 1):
        try:
            triangles = pg.triangulate_polygon(polygon)
            # Basic validation: n-gon should produce at least n-2 triangles
            min_triangles = len(polygon) - 2
            valid = len(triangles) >= min_triangles and all(len(t) == 3 for t in triangles)
            status = "✓" if valid else "✗"
            print(f"  Test {i}: {description} ({len(polygon)} vertices) → {len(triangles)} triangles {status}")
        except Exception as e:
            print(f"  Test {i}: {description} → ERROR: {e} ✗")
    
    # Test convex decomposition
    print("\nConvex Decomposition Tests:")
    for i, (polygon, description) in enumerate(test_polygons, 1):
        try:
            convex_parts = pg.convex_decomposition(polygon)
            valid = len(convex_parts) >= 1 and all(len(part) >= 3 for part in convex_parts)
            status = "✓" if valid else "✗"
            print(f"  Test {i}: {description} → {len(convex_parts)} convex parts {status}")
        except Exception as e:
            print(f"  Test {i}: {description} → ERROR: {e} ✗")

def performance_test():
    """Simple performance test"""
    print("\n" + "=" * 60)
    print("PERFORMANCE TEST")
    print("=" * 60)
    
    pg = PolyGeom()
    
    # Generate a polygon with many vertices
    n_vertices = 100
    polygon = []
    for i in range(n_vertices):
        angle = 2 * math.pi * i / n_vertices
        x = 5 * math.cos(angle)
        y = 5 * math.sin(angle)
        polygon.append((x, y))
    
    # Test distance calculation performance
    test_points = [(10 * (i/50 - 1), 10 * (i/50 - 1)) for i in range(100)]
    
    print(f"Testing distance calculation for {n_vertices}-vertex polygon with {len(test_points)} points...")
    
    start_time = time.perf_counter()
    for point in test_points:
        distance, closest = pg.point_to_polygon_distance(polygon, point)
    end_time = time.perf_counter()
    
    avg_time = (end_time - start_time) / len(test_points) * 1000  # Convert to ms
    print(f"  Average time per distance calculation: {avg_time:.3f}ms")
    
    # Test triangulation performance
    print(f"Testing triangulation for {n_vertices}-vertex polygon...")
    
    start_time = time.perf_counter()
    triangles = pg.triangulate_polygon(polygon)
    end_time = time.perf_counter()
    
    triangulation_time = (end_time - start_time) * 1000  # Convert to ms
    print(f"  Triangulation time: {triangulation_time:.3f}ms → {len(triangles)} triangles")

def main():
    """Run all tests"""
    print("PolyGeom C++ Library Test Suite")
    print("Using Python fallback implementation (Shapely-based)")
    
    try:
        test_basic_operations()
        test_geometric_operations() 
        test_polygon_operations()
        performance_test()
        
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        print("All tests completed! ✓")
        print("\nTo run more comprehensive tests:")
        print("  python -m pytest tests/test_polygeom_wrapper.py -v")
        print("\nTo run visual tests (requires matplotlib):")
        print("  python tests/visual_polygeom_test.py")
        print("\nTo run performance benchmarks:")
        print("  python tests/benchmark_polygeom.py")
        
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
