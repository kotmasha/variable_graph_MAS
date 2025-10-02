#!/usr/bin/env python3
"""
Visual testing script for polygeom operations
Shows geometric operations graphically to verify correctness
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import math

# Add utils directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from polygeom_wrapper import PolyGeom

def plot_polygon(ax, points, color='blue', alpha=0.3, label=None):
    """Plot a polygon"""
    if len(points) > 0:
        polygon = patches.Polygon(points, closed=True, alpha=alpha, color=color, label=label)
        ax.add_patch(polygon)
        
        # Also plot the outline
        points_closed = points + [points[0]]
        x_coords = [p[0] for p in points_closed]
        y_coords = [p[1] for p in points_closed]
        ax.plot(x_coords, y_coords, color=color, linewidth=2)

def plot_line(ax, points, color='red', label=None, linewidth=2):
    """Plot a line"""
    if len(points) >= 2:
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        ax.plot(x_coords, y_coords, color=color, linewidth=linewidth, label=label, marker='o')

def plot_ray(ax, origin, direction, length=5, color='green', label=None):
    """Plot a ray"""
    end_point = (origin[0] + length * direction[0], origin[1] + length * direction[1])
    ax.arrow(origin[0], origin[1], 
             length * direction[0], length * direction[1],
             head_width=0.1, head_length=0.1, fc=color, ec=color, label=label)

def test_point_to_polygon_distance():
    """Visual test of point-to-polygon distance calculations"""
    pg = PolyGeom()
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Point-to-Polygon Distance Tests', fontsize=16)
    
    # Test cases
    test_cases = [
        {
            'polygon': [(0, 0), (4, 0), (4, 4), (0, 4)],
            'points': [(2, -1), (6, 2), (2, 2), (-1, 2)],
            'title': 'Square with various points'
        },
        {
            'polygon': [(1, 1), (3, 1), (4, 3), (2, 4), (0, 3)],
            'points': [(2, 2), (5, 2), (2, 0), (-1, 3)],
            'title': 'Pentagon with test points'
        },
        {
            'polygon': [(2, 1), (3, 2), (2, 3), (1, 2)],
            'points': [(2, 2), (4, 2), (0, 2), (2, 0)],
            'title': 'Diamond shape'
        },
        {
            'polygon': [(0, 0), (3, 0), (4, 2), (3, 4), (0, 4), (-1, 2)],
            'points': [(2, 2), (5, 3), (-2, 2), (2, -1)],
            'title': 'Hexagon'
        }
    ]
    
    for i, test_case in enumerate(test_cases):
        ax = axes[i // 2, i % 2]
        polygon = test_case['polygon']
        points = test_case['points']
        
        # Plot polygon
        plot_polygon(ax, polygon, color='lightblue', alpha=0.3)
        
        # Test each point
        colors = ['red', 'green', 'orange', 'purple']
        for j, point in enumerate(points):
            distance, closest_point = pg.point_to_polygon_distance(polygon, point)
            
            # Plot test point
            ax.plot(point[0], point[1], 'o', color=colors[j], markersize=8, 
                   label=f'P{j+1}: d={distance:.2f}')
            
            # Plot closest point and connection line
            ax.plot(closest_point[0], closest_point[1], 's', color=colors[j], markersize=6)
            ax.plot([point[0], closest_point[0]], [point[1], closest_point[1]], 
                   '--', color=colors[j], alpha=0.7)
            
            # Add distance text
            mid_x = (point[0] + closest_point[0]) / 2
            mid_y = (point[1] + closest_point[1]) / 2
            ax.text(mid_x, mid_y, f'{distance:.2f}', fontsize=8, 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
        
        ax.set_title(test_case['title'])
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_aspect('equal')
    
    plt.tight_layout()
    return fig

def test_polygon_intersections():
    """Visual test of polygon intersection operations"""
    pg = PolyGeom()
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Polygon Intersection Tests', fontsize=16)
    
    # Base polygon for all tests
    polygon = [(1, 1), (5, 1), (5, 5), (1, 5)]
    
    # Ray intersection test
    ax = axes[0, 0]
    plot_polygon(ax, polygon, color='lightblue', alpha=0.3)
    
    ray_tests = [
        ((2, 3), (1, 0)),   # Right
        ((3, 3), (0, 1)),   # Up
        ((3, 3), (-1, -1)), # Down-left
        ((0, 3), (1, 0))    # From outside
    ]
    
    colors = ['red', 'green', 'orange', 'purple']
    for i, (origin, direction) in enumerate(ray_tests):
        try:
            intersection = pg.polygon_ray_intersection(polygon, origin, direction)
        except Exception as e:
            print(f"Ray test {i+1} failed: {e}")
            intersection = None
        
        # Normalize direction for display
        norm = math.sqrt(direction[0]**2 + direction[1]**2)
        if norm > 0:
            direction = (direction[0]/norm, direction[1]/norm)
        
        plot_ray(ax, origin, direction, length=3, color=colors[i])
        ax.plot(origin[0], origin[1], 'o', color=colors[i], markersize=6)
        
        if intersection:
            ax.plot(intersection[0], intersection[1], 's', color=colors[i], markersize=8)
            ax.text(intersection[0]+0.1, intersection[1]+0.1, f'Hit{i+1}', fontsize=8)
    
    ax.set_title('Ray-Polygon Intersection')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Line intersection test
    ax = axes[0, 1]
    plot_polygon(ax, polygon, color='lightblue', alpha=0.3)
    
    # Vertical line through center
    line_point = (3, 3)
    line_normal = (1, 0)  # Vertical line
    intersections = pg.polygon_line_intersection(polygon, line_point, line_normal)
    
    # Plot line (extend in both directions)
    ax.axvline(x=3, color='red', linestyle='--', alpha=0.7, label='Line')
    ax.plot(line_point[0], line_point[1], 'ro', markersize=6, label='Line point')
    
    # Plot intersections
    for i, point in enumerate(intersections):
        ax.plot(point[0], point[1], 'rs', markersize=8, label=f'Intersection {i+1}')
    
    ax.set_title('Line-Polygon Intersection')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_aspect('equal')
    
    # Halfplane intersection test
    ax = axes[1, 0]
    plot_polygon(ax, polygon, color='lightblue', alpha=0.3, label='Original')
    
    # Halfplane cutting through middle
    halfplane_point = (3, 3)
    halfplane_normal = (1, 0)  # Normal pointing right
    result = pg.polygon_halfplane_intersection(polygon, halfplane_point, halfplane_normal)
    
    if result:
        plot_polygon(ax, result, color='lightgreen', alpha=0.5, label='Intersection')
    
    # Draw halfplane boundary
    ax.axvline(x=3, color='red', linestyle='-', linewidth=2, label='Halfplane boundary')
    ax.arrow(3, 2, 0.5, 0, head_width=0.1, head_length=0.1, fc='red', ec='red')
    ax.text(3.3, 2, 'Normal', fontsize=8)
    
    ax.set_title('Halfplane-Polygon Intersection')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_aspect('equal')
    
    # Triangulation test
    ax = axes[1, 1]
    
    # Use a more complex polygon for triangulation
    complex_polygon = [(1, 1), (4, 1), (5, 2), (4, 4), (2, 4), (1, 3)]
    triangles = pg.triangulate_polygon(complex_polygon)
    
    plot_polygon(ax, complex_polygon, color='lightblue', alpha=0.2, label='Original')
    
    # Plot triangles with different colors
    triangle_colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink']
    for i, triangle in enumerate(triangles):
        color = triangle_colors[i % len(triangle_colors)]
        plot_polygon(ax, triangle, color=color, alpha=0.3, label=f'T{i+1}')
    
    ax.set_title(f'Triangulation ({len(triangles)} triangles)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    return fig

def test_mathematical_operations():
    """Visual test of mathematical operations"""
    pg = PolyGeom()
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Mathematical Operations Tests', fontsize=16)
    
    # Matrix operations visualization
    ax = axes[0]
    matrices = [
        [[1, 2], [3, 4]],
        [[2, 1], [1, 2]],
        [[1, 0], [0, 1]],
        [[1, 1], [1, 1]]
    ]
    
    determinants = [pg.matrix_determinant(m) for m in matrices]
    labels = ['[[1,2],[3,4]]', '[[2,1],[1,2]]', 'Identity', '[[1,1],[1,1]]']
    
    bars = ax.bar(range(len(matrices)), determinants, 
                 color=['red' if d < 0 else 'green' if d > 0 else 'gray' for d in determinants])
    ax.set_xlabel('Matrix')
    ax.set_ylabel('Determinant')
    ax.set_title('Matrix Determinants')
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for i, (bar, det) in enumerate(zip(bars, determinants)):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 0.05 * np.sign(height),
                f'{det:.2f}', ha='center', va='bottom' if height >= 0 else 'top')
    
    # Angle transformation test
    ax = axes[1]
    test_angles = np.linspace(-4*np.pi, 4*np.pi, 100)
    normalized_angles = [pg.angle_transformation(angle) for angle in test_angles]
    
    ax.plot(test_angles, normalized_angles, 'b-', linewidth=2, label='Normalized')
    ax.axhline(y=np.pi, color='r', linestyle='--', alpha=0.7, label='π')
    ax.axhline(y=-np.pi, color='r', linestyle='--', alpha=0.7, label='-π')
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_xlabel('Input Angle (radians)')
    ax.set_ylabel('Normalized Angle (radians)')
    ax.set_title('Angle Normalization to [-π, π]')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Matrix-vector multiplication visualization
    ax = axes[2]
    
    # Test matrix-vector operations
    matrix = [[2, 1], [1, 2]]
    test_vectors = [(1, 0), (0, 1), (1, 1), (-1, 1), (2, -1)]
    
    for i, vector in enumerate(test_vectors):
        result = pg.matrix_vector_multiply(matrix, list(vector))
        
        # Plot original vector
        ax.arrow(0, 0, vector[0], vector[1], 
                head_width=0.1, head_length=0.1, 
                fc=f'C{i}', ec=f'C{i}', alpha=0.7, label=f'v{i+1}')
        
        # Plot transformed vector
        ax.arrow(0, 0, result[0], result[1], 
                head_width=0.1, head_length=0.1, 
                fc=f'C{i}', ec=f'C{i}', linestyle='--', 
                alpha=1.0)
        
        # Add labels
        ax.text(vector[0] + 0.1, vector[1] + 0.1, 
               f'({vector[0]},{vector[1]})', fontsize=8)
        ax.text(result[0] + 0.1, result[1] + 0.1, 
               f'({result[0]:.1f},{result[1]:.1f})', fontsize=8)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Matrix-Vector Multiplication\nMatrix [[2,1],[1,2]]')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.legend()
    
    plt.tight_layout()
    return fig

def run_all_visual_tests():
    """Run all visual tests and save plots"""
    print("Running visual tests for polygeom...")
    
    # Run tests
    fig1 = test_point_to_polygon_distance()
    fig2 = test_polygon_intersections()
    fig3 = test_mathematical_operations()
    
    # Save plots
    fig1.savefig('polygeom_distance_tests.png', dpi=150, bbox_inches='tight')
    fig2.savefig('polygeom_intersection_tests.png', dpi=150, bbox_inches='tight')
    fig3.savefig('polygeom_math_tests.png', dpi=150, bbox_inches='tight')
    
    print("Visual tests completed. Plots saved as:")
    print("- polygeom_distance_tests.png")
    print("- polygeom_intersection_tests.png")  
    print("- polygeom_math_tests.png")
    
    # Show plots
    plt.show()

if __name__ == "__main__":
    run_all_visual_tests()
