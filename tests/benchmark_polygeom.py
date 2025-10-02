#!/usr/bin/env python3
"""
Performance benchmark tests for polygeom operations
Measures execution time and validates outputs for various polygon sizes
"""

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple
import math
import statistics

# Add utils directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from polygeom_wrapper import PolyGeom

def generate_regular_polygon(n_vertices: int, radius: float = 2.0, center: Tuple[float, float] = (0, 0)) -> List[Tuple[float, float]]:
    """Generate a regular polygon with n vertices"""
    points = []
    for i in range(n_vertices):
        angle = 2 * math.pi * i / n_vertices
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.append((x, y))
    return points

def generate_random_polygon(n_vertices: int, radius: float = 2.0) -> List[Tuple[float, float]]:
    """Generate a random polygon (star-shaped to ensure validity)"""
    angles = sorted([2 * math.pi * np.random.random() for _ in range(n_vertices)])
    points = []
    for angle in angles:
        r = radius * (0.5 + 0.5 * np.random.random())  # Random radius between 0.5 and 1.0 of max
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        points.append((x, y))
    return points

def benchmark_distance_calculation():
    """Benchmark point-to-polygon distance calculation"""
    print("Benchmarking point-to-polygon distance calculation...")
    
    pg = PolyGeom()
    polygon_sizes = [4, 8, 16, 32, 64, 128]
    num_points = 100
    results = {}
    
    for n_vertices in polygon_sizes:
        print(f"  Testing {n_vertices}-vertex polygon...")
        
        # Generate test polygon
        polygon = generate_regular_polygon(n_vertices, radius=5.0)
        
        # Generate test points
        test_points = [(10 * np.random.random() - 5, 10 * np.random.random() - 5) 
                      for _ in range(num_points)]
        
        # Benchmark
        times = []
        distances = []
        
        for point in test_points:
            start_time = time.perf_counter()
            distance, closest = pg.point_to_polygon_distance(polygon, point)
            end_time = time.perf_counter()
            
            times.append(end_time - start_time)
            distances.append(distance)
        
        results[n_vertices] = {
            'avg_time': statistics.mean(times),
            'min_time': min(times),
            'max_time': max(times),
            'std_time': statistics.stdev(times),
            'avg_distance': statistics.mean(distances),
            'num_points': num_points
        }
    
    return results

def benchmark_intersection_operations():
    """Benchmark various intersection operations"""
    print("Benchmarking intersection operations...")
    
    pg = PolyGeom()
    polygon_sizes = [4, 8, 16, 32, 64]
    num_tests = 50
    results = {}
    
    for n_vertices in polygon_sizes:
        print(f"  Testing {n_vertices}-vertex polygon...")
        
        polygon = generate_regular_polygon(n_vertices, radius=5.0)
        
        # Test ray intersections
        ray_times = []
        for _ in range(num_tests):
            origin = (10 * np.random.random() - 5, 10 * np.random.random() - 5)
            direction = (np.random.random() - 0.5, np.random.random() - 0.5)
            
            start_time = time.perf_counter()
            intersection = pg.polygon_ray_intersection(polygon, origin, direction)
            end_time = time.perf_counter()
            
            ray_times.append(end_time - start_time)
        
        # Test line intersections
        line_times = []
        for _ in range(num_tests):
            line_point = (10 * np.random.random() - 5, 10 * np.random.random() - 5)
            line_normal = (np.random.random() - 0.5, np.random.random() - 0.5)
            
            start_time = time.perf_counter()
            intersections = pg.polygon_line_intersection(polygon, line_point, line_normal)
            end_time = time.perf_counter()
            
            line_times.append(end_time - start_time)
        
        # Test triangulation
        start_time = time.perf_counter()
        triangles = pg.triangulate_polygon(polygon)
        triangulation_time = time.perf_counter() - start_time
        
        results[n_vertices] = {
            'ray_avg_time': statistics.mean(ray_times),
            'ray_std_time': statistics.stdev(ray_times),
            'line_avg_time': statistics.mean(line_times),
            'line_std_time': statistics.stdev(line_times),
            'triangulation_time': triangulation_time,
            'num_triangles': len(triangles)
        }
    
    return results

def benchmark_mathematical_operations():
    """Benchmark mathematical operations"""
    print("Benchmarking mathematical operations...")
    
    pg = PolyGeom()
    num_tests = 10000
    
    # Matrix determinant benchmark
    matrices = [[[np.random.random(), np.random.random()], 
                 [np.random.random(), np.random.random()]] for _ in range(num_tests)]
    
    start_time = time.perf_counter()
    determinants = [pg.matrix_determinant(m) for m in matrices]
    det_time = time.perf_counter() - start_time
    
    # Matrix-vector multiplication benchmark
    matrix = [[2, 1], [1, 2]]
    vectors = [[np.random.random(), np.random.random()] for _ in range(num_tests)]
    
    start_time = time.perf_counter()
    results_mv = [pg.matrix_vector_multiply(matrix, v) for v in vectors]
    mv_time = time.perf_counter() - start_time
    
    # Angle transformation benchmark
    angles = [2 * math.pi * np.random.random() - math.pi for _ in range(num_tests)]
    
    start_time = time.perf_counter()
    normalized_angles = [pg.angle_transformation(a) for a in angles]
    angle_time = time.perf_counter() - start_time
    
    return {
        'determinant': {'total_time': det_time, 'avg_time': det_time / num_tests, 'num_tests': num_tests},
        'matrix_vector': {'total_time': mv_time, 'avg_time': mv_time / num_tests, 'num_tests': num_tests},
        'angle_transform': {'total_time': angle_time, 'avg_time': angle_time / num_tests, 'num_tests': num_tests}
    }

def stress_test_complex_polygons():
    """Stress test with very complex polygons"""
    print("Running stress test with complex polygons...")
    
    pg = PolyGeom()
    complex_sizes = [100, 200, 500, 1000]
    results = {}
    
    for n_vertices in complex_sizes:
        print(f"  Testing {n_vertices}-vertex polygon...")
        
        try:
            # Generate complex polygon
            polygon = generate_random_polygon(n_vertices, radius=10.0)
            
            # Test distance calculation
            test_point = (0, 0)
            start_time = time.perf_counter()
            distance, closest = pg.point_to_polygon_distance(polygon, test_point)
            distance_time = time.perf_counter() - start_time
            
            # Test triangulation
            start_time = time.perf_counter()
            triangles = pg.triangulate_polygon(polygon)
            triangulation_time = time.perf_counter() - start_time
            
            results[n_vertices] = {
                'distance_time': distance_time,
                'triangulation_time': triangulation_time,
                'num_triangles': len(triangles),
                'success': True
            }
            
        except Exception as e:
            results[n_vertices] = {
                'error': str(e),
                'success': False
            }
    
    return results

def plot_benchmark_results(distance_results, intersection_results, math_results, stress_results):
    """Plot benchmark results"""
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('PolyGeom Performance Benchmarks', fontsize=16)
    
    # Distance calculation performance
    ax = axes[0, 0]
    sizes = list(distance_results.keys())
    avg_times = [distance_results[s]['avg_time'] * 1000 for s in sizes]  # Convert to ms
    std_times = [distance_results[s]['std_time'] * 1000 for s in sizes]
    
    ax.errorbar(sizes, avg_times, yerr=std_times, marker='o', capsize=5)
    ax.set_xlabel('Polygon Vertices')
    ax.set_ylabel('Average Time (ms)')
    ax.set_title('Point-to-Polygon Distance Performance')
    ax.grid(True, alpha=0.3)
    ax.set_xscale('log')
    ax.set_yscale('log')
    
    # Intersection operations performance
    ax = axes[0, 1]
    sizes = list(intersection_results.keys())
    ray_times = [intersection_results[s]['ray_avg_time'] * 1000 for s in sizes]
    line_times = [intersection_results[s]['line_avg_time'] * 1000 for s in sizes]
    triangulation_times = [intersection_results[s]['triangulation_time'] * 1000 for s in sizes]
    
    ax.plot(sizes, ray_times, 'o-', label='Ray Intersection', marker='o')
    ax.plot(sizes, line_times, 's-', label='Line Intersection', marker='s')
    ax.plot(sizes, triangulation_times, '^-', label='Triangulation', marker='^')
    
    ax.set_xlabel('Polygon Vertices')
    ax.set_ylabel('Average Time (ms)')
    ax.set_title('Intersection Operations Performance')
    ax.grid(True, alpha=0.3)
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.legend()
    
    # Mathematical operations performance
    ax = axes[1, 0]
    operations = list(math_results.keys())
    op_times = [math_results[op]['avg_time'] * 1000000 for op in operations]  # Convert to microseconds
    
    bars = ax.bar(operations, op_times, color=['red', 'green', 'blue'])
    ax.set_ylabel('Average Time (μs)')
    ax.set_title('Mathematical Operations Performance')
    ax.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, time_val in zip(bars, op_times):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{time_val:.2f}', ha='center', va='bottom')
    
    # Stress test results
    ax = axes[1, 1]
    stress_sizes = [s for s in stress_results.keys() if stress_results[s]['success']]
    stress_distance_times = [stress_results[s]['distance_time'] * 1000 for s in stress_sizes]
    stress_triangulation_times = [stress_results[s]['triangulation_time'] * 1000 for s in stress_sizes]
    
    ax.plot(stress_sizes, stress_distance_times, 'o-', label='Distance Calculation', marker='o')
    ax.plot(stress_sizes, stress_triangulation_times, 's-', label='Triangulation', marker='s')
    
    ax.set_xlabel('Polygon Vertices')
    ax.set_ylabel('Time (ms)')
    ax.set_title('Stress Test - Complex Polygons')
    ax.grid(True, alpha=0.3)
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.legend()
    
    plt.tight_layout()
    return fig

def run_all_benchmarks():
    """Run all benchmark tests"""
    print("Starting PolyGeom performance benchmarks...")
    print("=" * 60)
    
    # Run benchmarks
    distance_results = benchmark_distance_calculation()
    intersection_results = benchmark_intersection_operations()
    math_results = benchmark_mathematical_operations()
    stress_results = stress_test_complex_polygons()
    
    # Print summary
    print("\n" + "=" * 60)
    print("BENCHMARK SUMMARY")
    print("=" * 60)
    
    print("\nDistance Calculation Performance:")
    for size, result in distance_results.items():
        print(f"  {size:3d} vertices: {result['avg_time']*1000:.3f}ms ± {result['std_time']*1000:.3f}ms")
    
    print("\nIntersection Operations Performance:")
    for size, result in intersection_results.items():
        print(f"  {size:3d} vertices:")
        print(f"    Ray intersection:  {result['ray_avg_time']*1000:.3f}ms")
        print(f"    Line intersection: {result['line_avg_time']*1000:.3f}ms")
        print(f"    Triangulation:     {result['triangulation_time']*1000:.3f}ms ({result['num_triangles']} triangles)")
    
    print("\nMathematical Operations Performance:")
    for op, result in math_results.items():
        print(f"  {op.replace('_', ' ').title()}: {result['avg_time']*1000000:.3f}μs per operation")
    
    print("\nStress Test Results:")
    for size, result in stress_results.items():
        if result['success']:
            print(f"  {size:4d} vertices: Distance={result['distance_time']*1000:.2f}ms, "
                  f"Triangulation={result['triangulation_time']*1000:.2f}ms ({result['num_triangles']} triangles)")
        else:
            print(f"  {size:4d} vertices: FAILED - {result['error']}")
    
    # Plot results
    fig = plot_benchmark_results(distance_results, intersection_results, math_results, stress_results)
    fig.savefig('polygeom_benchmarks.png', dpi=150, bbox_inches='tight')
    print(f"\nBenchmark plot saved as: polygeom_benchmarks.png")
    
    plt.show()
    return distance_results, intersection_results, math_results, stress_results

if __name__ == "__main__":
    run_all_benchmarks()
