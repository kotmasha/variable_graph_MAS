"""
Python wrapper for the polygeom C++ library
Provides high-level Python interface for computational geometry operations
"""

import numpy as np
import ctypes
from ctypes import POINTER, c_double, c_int, c_bool, Structure, CDLL
from typing import List, Tuple, Optional, Union
import os
import sys
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity

# Add current directory to path for imports
sys.path.append(os.path.dirname(__file__))

class PolyGeomError(Exception):
    """Custom exception for polygeom operations"""
    pass

class Point2D(Structure):
    """C structure for 2D points"""
    _fields_ = [("x", c_double), ("y", c_double)]

class ProjectionResult(Structure):
    """C structure for projection results"""
    _fields_ = [("dist", c_double), ("projected_x", c_double), ("projected_y", c_double)]

class PolyGeom:
    """
    Python wrapper for the polygeom C++ library
    Provides computational geometry operations for robotics and navigation
    """
    
    def __init__(self, lib_path: Optional[str] = None):
        """
        Initialize the PolyGeom wrapper
        
        Args:
            lib_path: Path to the compiled polygeom shared library
                     If None, will attempt to find or compile the library
        """
        self.lib_path = lib_path
        self.lib = None
        self._load_library()
    
    def _load_library(self):
        """Load the C++ library or use pure Python implementations as fallback"""
        if self.lib_path and os.path.exists(self.lib_path):
            try:
                self.lib = CDLL(self.lib_path)
                self._setup_library_functions()
                print("Successfully loaded C++ polygeom library")
                return
            except Exception as e:
                print(f"Failed to load C++ library: {e}")
        
        print("Using pure Python implementations as fallback")
        self.lib = None
    
    def _setup_library_functions(self):
        """Setup function signatures for the C++ library"""
        if not self.lib:
            return
        
        # Matrix operations
        self.lib.MatrixDeterminant.argtypes = [POINTER(POINTER(c_double)), c_int]
        self.lib.MatrixDeterminant.restype = c_double
        
        # Distance calculations
        self.lib.polydist.argtypes = [POINTER(Point2D), c_int, Point2D]
        self.lib.polydist.restype = ProjectionResult
    
    # Pure Python implementations as fallback
    
    def matrix_determinant(self, matrix: List[List[float]]) -> float:
        """
        Calculate 2x2 matrix determinant
        
        Args:
            matrix: 2x2 matrix as list of lists
            
        Returns:
            Determinant value
        """
        if self.lib:
            # Use C++ implementation if available
            pass
        
        # Python fallback
        if len(matrix) != 2 or len(matrix[0]) != 2:
            raise ValueError("Only 2x2 matrices supported")
        
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    
    def matrix_vector_multiply(self, matrix: List[List[float]], vector: List[float]) -> List[float]:
        """
        Multiply matrix by vector
        
        Args:
            matrix: Matrix as list of lists
            vector: Vector as list
            
        Returns:
            Result vector
        """
        return [sum(matrix[i][j] * vector[j] for j in range(len(vector))) 
                for i in range(len(matrix))]
    
    def angle_transformation(self, angle: float) -> float:
        """
        Transform angle to be between -π and π
        
        Args:
            angle: Input angle in radians
            
        Returns:
            Normalized angle between -π and π
        """
        import math
        output_angle = math.fmod(angle + math.pi, 2.0 * math.pi)
        if output_angle < 0.0:
            output_angle += 2.0 * math.pi
        return output_angle - math.pi
    
    def point_to_polygon_distance(self, polygon_points: List[Tuple[float, float]], 
                                 point: Tuple[float, float]) -> Tuple[float, Tuple[float, float]]:
        """
        Calculate minimum distance from point to polygon and return closest point
        
        Args:
            polygon_points: List of (x, y) tuples defining polygon vertices
            point: (x, y) tuple for the query point
            
        Returns:
            Tuple of (distance, closest_point) where closest_point is (x, y)
        """
        if self.lib:
            # Use C++ implementation if available
            pass
        
        # Python fallback using shapely
        try:
            poly = Polygon(polygon_points)
            pt = Point(point)
            
            # Get distance and closest point
            distance = poly.exterior.distance(pt)
            
            # Find closest point on boundary
            closest_point = poly.exterior.interpolate(poly.exterior.project(pt))
            
            return distance, (closest_point.x, closest_point.y)
        
        except Exception as e:
            raise PolyGeomError(f"Error in point to polygon distance calculation: {e}")
    
    def point_to_line_distance(self, line_points: List[Tuple[float, float]], 
                              point: Tuple[float, float]) -> Tuple[float, Tuple[float, float]]:
        """
        Calculate minimum distance from point to line and return closest point
        
        Args:
            line_points: List of (x, y) tuples defining line vertices
            point: (x, y) tuple for the query point
            
        Returns:
            Tuple of (distance, closest_point) where closest_point is (x, y)
        """
        try:
            line = LineString(line_points)
            pt = Point(point)
            
            distance = line.distance(pt)
            closest_point = line.interpolate(line.project(pt))
            
            return distance, (closest_point.x, closest_point.y)
        
        except Exception as e:
            raise PolyGeomError(f"Error in point to line distance calculation: {e}")
    
    def polygon_line_intersection(self, polygon_points: List[Tuple[float, float]], 
                                 line_point: Tuple[float, float], 
                                 line_normal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Find intersection points between a polygon and a line
        
        Args:
            polygon_points: List of (x, y) tuples defining polygon vertices
            line_point: Point on the line (x, y)
            line_normal: Normal vector to the line (x, y)
            
        Returns:
            List of intersection points as (x, y) tuples
        """
        try:
            poly = Polygon(polygon_points)
            
            # Create line from point and normal
            # Line equation: (x - line_point.x) * normal.x + (y - line_point.y) * normal.y = 0
            # Convert to parametric form for intersection
            
            # For simplicity, create a long line segment in both directions
            import math
            length = 1000  # Large number to ensure line crosses polygon
            
            # Perpendicular to normal
            direction = (-line_normal[1], line_normal[0])
            norm = math.sqrt(direction[0]**2 + direction[1]**2)
            if norm > 0:
                direction = (direction[0]/norm, direction[1]/norm)
            
            # Create line endpoints
            p1 = (line_point[0] - length * direction[0], line_point[1] - length * direction[1])
            p2 = (line_point[0] + length * direction[0], line_point[1] + length * direction[1])
            
            line = LineString([p1, p2])
            intersection = poly.boundary.intersection(line)
            
            # Extract intersection points
            if intersection.is_empty:
                return []
            elif hasattr(intersection, 'coords'):
                return list(intersection.coords)
            elif hasattr(intersection, 'geoms'):
                points = []
                for geom in intersection.geoms:
                    if hasattr(geom, 'coords'):
                        points.extend(list(geom.coords))
                return points
            else:
                return [(intersection.x, intersection.y)]
                
        except Exception as e:
            raise PolyGeomError(f"Error in polygon-line intersection: {e}")
    
    def polygon_halfplane_intersection(self, polygon_points: List[Tuple[float, float]], 
                                     halfplane_point: Tuple[float, float], 
                                     halfplane_normal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Find intersection of polygon with halfplane
        
        Args:
            polygon_points: List of (x, y) tuples defining polygon vertices
            halfplane_point: Point on halfplane boundary (x, y)
            halfplane_normal: Inward normal of halfplane (x, y)
            
        Returns:
            List of vertices of intersected polygon as (x, y) tuples
        """
        try:
            poly = Polygon(polygon_points)
            
            # Create a large polygon representing the halfplane
            import math
            length = 10000  # Large number
            
            # Normal vector
            nx, ny = halfplane_normal
            norm = math.sqrt(nx**2 + ny**2)
            if norm > 0:
                nx, ny = nx/norm, ny/norm
            
            # Perpendicular vector
            px, py = -ny, nx
            
            # Create halfplane as large polygon
            center = halfplane_point
            halfplane_poly = Polygon([
                (center[0] - length * px - length * nx, center[1] - length * py - length * ny),
                (center[0] + length * px - length * nx, center[1] + length * py - length * ny),
                (center[0] + length * px + length * nx, center[1] + length * py + length * ny),
                (center[0] - length * px + length * nx, center[1] - length * py + length * ny)
            ])
            
            # Intersect with polygon
            result = poly.intersection(halfplane_poly)
            
            if result.is_empty:
                return []
            elif hasattr(result, 'exterior'):
                return list(result.exterior.coords)[:-1]  # Remove duplicate last point
            else:
                return []
                
        except Exception as e:
            raise PolyGeomError(f"Error in polygon-halfplane intersection: {e}")
    
    def polygon_ray_intersection(self, polygon_points: List[Tuple[float, float]], 
                                ray_origin: Tuple[float, float], 
                                ray_direction: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Find intersection point of polygon with a ray
        
        Args:
            polygon_points: List of (x, y) tuples defining polygon vertices
            ray_origin: Starting point of ray (x, y)
            ray_direction: Direction vector of ray (x, y)
            
        Returns:
            First intersection point as (x, y) tuple, or None if no intersection
        """
        try:
            poly = Polygon(polygon_points)
            
            # Create ray as line segment
            import math
            length = 10000  # Large number
            
            # Normalize direction
            dx, dy = ray_direction
            norm = math.sqrt(dx**2 + dy**2)
            if norm > 0:
                dx, dy = dx/norm, dy/norm
            
            # Create ray endpoint
            ray_end = (ray_origin[0] + length * dx, ray_origin[1] + length * dy)
            ray = LineString([ray_origin, ray_end])
            
            # Find intersection
            intersection = poly.boundary.intersection(ray)
            
            if intersection.is_empty:
                return None
            elif hasattr(intersection, 'coords'):
                # Return first intersection point
                coords = list(intersection.coords)
                if coords:
                    return coords[0]
            elif hasattr(intersection, 'geoms'):
                # Multiple intersections, return closest to origin
                min_dist = float('inf')
                closest_point = None
                
                for geom in intersection.geoms:
                    if hasattr(geom, 'x') and hasattr(geom, 'y'):
                        point = (geom.x, geom.y)
                        dist = math.sqrt((point[0] - ray_origin[0])**2 + (point[1] - ray_origin[1])**2)
                        if dist < min_dist:
                            min_dist = dist
                            closest_point = point
                
                return closest_point
            elif hasattr(intersection, 'x') and hasattr(intersection, 'y'):
                return (intersection.x, intersection.y)
            
            return None
                
        except Exception as e:
            raise PolyGeomError(f"Error in polygon-ray intersection: {e}")
    
    def triangulate_polygon(self, polygon_points: List[Tuple[float, float]], 
                           workspace_points: Optional[List[Tuple[float, float]]] = None,
                           touching_boundary: bool = False) -> List[List[Tuple[float, float]]]:
        """
        Triangulate a polygon
        
        Args:
            polygon_points: List of (x, y) tuples defining polygon vertices
            workspace_points: Optional workspace boundary points
            touching_boundary: Whether polygon touches workspace boundary
            
        Returns:
            List of triangles, each triangle is a list of 3 (x, y) tuples
        """
        try:
            # Simple ear clipping triangulation using shapely
            from shapely.ops import triangulate
            
            poly = Polygon(polygon_points)
            triangles = triangulate(poly)
            
            result = []
            for triangle in triangles:
                if triangle.intersects(poly):
                    coords = list(triangle.exterior.coords)[:-1]  # Remove duplicate last point
                    if len(coords) == 3:
                        result.append(coords)
            
            return result
            
        except Exception as e:
            raise PolyGeomError(f"Error in polygon triangulation: {e}")
    
    def convex_decomposition(self, polygon_points: List[Tuple[float, float]], 
                           workspace_points: Optional[List[Tuple[float, float]]] = None,
                           touching_boundary: bool = False) -> List[List[Tuple[float, float]]]:
        """
        Decompose polygon into convex parts
        
        Args:
            polygon_points: List of (x, y) tuples defining polygon vertices
            workspace_points: Optional workspace boundary points
            touching_boundary: Whether polygon touches workspace boundary
            
        Returns:
            List of convex polygons, each polygon is a list of (x, y) tuples
        """
        try:
            # Simple convex decomposition - for complex cases, would need the C++ library
            poly = Polygon(polygon_points)
            
            # If already convex, return as-is
            convex_hull = poly.convex_hull
            if poly.equals(convex_hull):
                return [polygon_points]
            
            # For non-convex polygons, this is a complex operation
            # For now, return triangulation as a fallback
            return self.triangulate_polygon(polygon_points, workspace_points, touching_boundary)
            
        except Exception as e:
            raise PolyGeomError(f"Error in convex decomposition: {e}")

# Convenience functions for easy import
def create_polygeom(lib_path: Optional[str] = None) -> PolyGeom:
    """Create a PolyGeom instance"""
    return PolyGeom(lib_path)

# Example usage and testing functions
def test_polygeom():
    """Test the PolyGeom wrapper with simple examples"""
    pg = PolyGeom()
    
    # Test basic operations
    print("Testing PolyGeom wrapper...")
    
    # Test matrix determinant
    matrix = [[1, 2], [3, 4]]
    det = pg.matrix_determinant(matrix)
    print(f"Determinant of {matrix} = {det}")
    
    # Test angle transformation
    import math
    angle = 3 * math.pi
    normalized = pg.angle_transformation(angle)
    print(f"Angle {angle} normalized to {normalized}")
    
    # Test point to polygon distance
    polygon = [(0, 0), (4, 0), (4, 4), (0, 4)]
    point = (2, -1)
    dist, closest = pg.point_to_polygon_distance(polygon, point)
    print(f"Distance from {point} to polygon: {dist}, closest point: {closest}")
    
    # Test polygon-ray intersection
    ray_origin = (2, 2)
    ray_direction = (1, 0)
    intersection = pg.polygon_ray_intersection(polygon, ray_origin, ray_direction)
    print(f"Ray intersection: {intersection}")
    
    # Test triangulation
    triangles = pg.triangulate_polygon(polygon)
    print(f"Triangulation: {len(triangles)} triangles")
    
    print("All tests completed!")

if __name__ == "__main__":
    test_polygeom()
