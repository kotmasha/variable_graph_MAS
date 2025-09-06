"""
Simple integration guide for using polygeom wrapper in existing navigation code

This file shows how to integrate the polygeom wrapper with your existing
Network.py and Environment.py files in the src directory.
"""

import sys
import os
import numpy as np
from typing import List, Tuple, Optional

# Add utils directory to path
sys.path.append(os.path.dirname(__file__))

# Import the polygeom wrapper
from polygeom_wrapper import PolyGeom, PolyGeomError

def demonstrate_basic_usage():
    """Demonstrate basic usage of the polygeom wrapper"""
    
    print("=== PolyGeom Wrapper Basic Usage Demo ===\n")
    
    # Create polygeom instance
    pg = PolyGeom()
    
    # Define a simple polygon (square)
    polygon_points = [(0, 0), (4, 0), (4, 4), (0, 4)]
    
    # Test point-to-polygon distance
    test_point = (2, -1)
    distance, closest_point = pg.point_to_polygon_distance(polygon_points, test_point)
    print(f"Distance from point {test_point} to polygon: {distance:.2f}")
    print(f"Closest point on polygon: ({closest_point[0]:.2f}, {closest_point[1]:.2f})")
    
    # Test polygon-ray intersection
    ray_origin = (2, 2)
    ray_direction = (1, 0)
    intersection = pg.polygon_ray_intersection(polygon_points, ray_origin, ray_direction)
    print(f"Ray from {ray_origin} in direction {ray_direction} intersects at: {intersection}")
    
    # Test matrix operations
    matrix = [[1, 2], [3, 4]]
    det = pg.matrix_determinant(matrix)
    print(f"Determinant of {matrix} = {det}")
    
    print("\n" + "="*60 + "\n")

def enhanced_navigation_example():
    """Show how to enhance navigation using polygeom"""
    
    print("=== Enhanced Navigation Example ===\n")
    
    pg = PolyGeom()
    
    # Define workspace and obstacles
    workspace = [(0, 0), (10, 0), (10, 10), (0, 10)]
    obstacles = [
        [(3, 3), (4, 3), (4, 4), (3, 4)],  # Small square
        [(6, 6), (8, 6), (8, 8), (6, 8)]   # Another square
    ]
    
    def is_point_safe(point: Tuple[float, float], safety_margin: float = 0.2) -> bool:
        """Check if point is safe using polygeom"""
        try:
            from shapely.geometry import Polygon, Point
            
            # Check workspace bounds
            workspace_poly = Polygon(workspace)
            pt = Point(point)
            if not workspace_poly.contains(pt):
                return False
            
            # Check obstacle distances
            for obstacle in obstacles:
                distance, _ = pg.point_to_polygon_distance(obstacle, point)
                if distance < safety_margin:
                    return False
            return True
        except:
            return False
    
    # Test safe points
    test_points = [(1, 1), (3.5, 3.5), (5, 5), (9, 9)]
    
    for point in test_points:
        safe = is_point_safe(point)
        status = "SAFE" if safe else "UNSAFE"
        print(f"Point {point}: {status}")
    
    print("\n" + "="*60 + "\n")

def integration_patterns():
    """Show different patterns for integrating with existing code"""
    
    print("=== Integration Patterns ===\n")
    
    # Pattern 1: Add polygeom to existing environment class
    print("Pattern 1: Enhance existing Environment class")
    print("""
# In your Environment.py file:
from utils.polygeom_wrapper import PolyGeom

class sphereworldEnv(environment):
    def __init__(self, outerbounds, obstacleData):
        super().__init__(outerbounds, obstacleData)
        # Add polygeom instance
        self.polygeom = PolyGeom()
        
    def enhanced_safety_check(self, state, safety_margin=0.1):
        \"\"\"Enhanced safety check using polygeom\"\"\"
        state_point = (state[0, 0], state[1, 0])
        
        for i in range(self.obstacleNum):
            # Convert obstacle to polygon points
            center = self.obstacleCenters[i]
            radius = self.obstacleRadii[i]
            
            # Create circle as polygon (simplified)
            import math
            n_points = 16
            circle_points = []
            for j in range(n_points):
                angle = j * 2 * math.pi / n_points
                x = center[0] + radius * math.cos(angle)
                y = center[1] + radius * math.sin(angle)
                circle_points.append((x, y))
            
            # Check distance using polygeom
            distance, _ = self.polygeom.point_to_polygon_distance(circle_points, state_point)
            if distance < safety_margin:
                return False
        return True
""")
    
    # Pattern 2: Add to navigation
    print("\nPattern 2: Enhance Navigation functions")
    print("""
# In your Navigation.py file:
from utils.polygeom_wrapper import PolyGeom

class navigation:
    def __init__(self):
        self.polygeom = PolyGeom()
    
    def enhanced_obstacle_avoidance(self, state, goal, obstacle_polygons):
        \"\"\"Enhanced obstacle avoidance using polygeom\"\"\"
        state_point = (state[0, 0], state[1, 0])
        
        # Compute repulsive forces from obstacles
        total_repulsion = np.array([0.0, 0.0])
        
        for obstacle in obstacle_polygons:
            distance, closest_point = self.polygeom.point_to_polygon_distance(obstacle, state_point)
            
            if distance < 2.0:  # Influence radius
                repulsion_dir = np.array(state_point) - np.array(closest_point)
                if np.linalg.norm(repulsion_dir) > 0:
                    repulsion_dir = repulsion_dir / np.linalg.norm(repulsion_dir)
                    repulsion_magnitude = 1.0 / (distance + 0.1)
                    total_repulsion += repulsion_magnitude * repulsion_dir
        
        # Combine with attractive force to goal
        attractive = goal - state
        if np.linalg.norm(attractive) > 0:
            attractive = attractive / np.linalg.norm(attractive)
        
        combined_force = attractive.flatten() + 0.3 * total_repulsion
        return combined_force.reshape(2, 1)
""")
    
    # Pattern 3: Mixin approach
    print("\nPattern 3: Mixin approach for existing classes")
    print("""
# Create a mixin class:
class PolyGeomMixin:
    def __init__(self):
        super().__init__()
        self.polygeom = PolyGeom()
    
    def compute_polygon_distance(self, polygon_points, point):
        return self.polygeom.point_to_polygon_distance(polygon_points, point)
    
    def check_line_intersection(self, polygon_points, line_point, line_normal):
        return self.polygeom.polygon_line_intersection(polygon_points, line_point, line_normal)

# Then use it in your existing classes:
class EnhancedSphereworldEnv(PolyGeomMixin, sphereworldEnv):
    def __init__(self, outerbounds, obstacleData):
        super().__init__(outerbounds, obstacleData)
        # Now you have access to polygeom methods
""")
    
    print("\n" + "="*60 + "\n")

def simple_example_for_your_code():
    """Simple example you can directly use in your existing code"""
    
    print("=== Simple Example for Direct Use ===\n")
    
    print("Here's a simple function you can add to your existing files:")
    print("""
# Add this to the top of your Network.py or Environment.py:
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from polygeom_wrapper import PolyGeom

# Add this as a method to your existing class:
def enhanced_distance_check(self, state, obstacle_centers, obstacle_radii):
    \"\"\"
    Enhanced distance checking using polygeom
    state: numpy array [x, y] 
    obstacle_centers: list of [x, y] positions
    obstacle_radii: list of radii
    \"\"\"
    try:
        pg = PolyGeom()  # Create polygeom instance
        state_point = (float(state[0]), float(state[1]))
        
        min_distance = float('inf')
        
        for i, (center, radius) in enumerate(zip(obstacle_centers, obstacle_radii)):
            # Create circle as polygon
            import math
            n_points = 12
            circle_points = []
            for j in range(n_points):
                angle = j * 2 * math.pi / n_points
                x = center[0] + radius * math.cos(angle) 
                y = center[1] + radius * math.sin(angle)
                circle_points.append((x, y))
            
            # Use polygeom to compute distance
            distance, closest = pg.point_to_polygon_distance(circle_points, state_point)
            min_distance = min(min_distance, distance)
        
        return min_distance
    except Exception as e:
        # Fallback to your existing method
        return self.your_existing_distance_method(state, obstacle_centers, obstacle_radii)
""")
    
    print("\nThen call it in your existing navigation functions like:")
    print("""
# In your navfSphere method:
min_dist = self.enhanced_distance_check(state, self.obstacleCenters, self.obstacleRadii)
if min_dist < safety_threshold:
    # Apply stronger repulsive force
    pass
""")

if __name__ == "__main__":
    demonstrate_basic_usage()
    enhanced_navigation_example()
    integration_patterns()
    simple_example_for_your_code()
    
    print("=== Testing the polygeom wrapper ===")
    # Test the wrapper
    from polygeom_wrapper import test_polygeom
    test_polygeom()
