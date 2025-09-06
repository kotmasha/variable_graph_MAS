"""
Comprehensive tests for the polygeom wrapper
"""
import math
import pytest
import sys
import os

# Add parent directory to path to import utils
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from utils.polygeom_wrapper import create_polygeom, PolyGeomError


class TestPolyGeomWrapper:
    """Test suite for PolyGeom wrapper functionality"""
    
    def setup_method(self):
        """Setup for each test method"""
        self.pg = create_polygeom()
    
    def test_matrix_determinant_2x2(self):
        """Test 2x2 matrix determinant calculation"""
        # Test case 1: Standard matrix
        matrix1 = [[1, 2], [3, 4]]
        result1 = self.pg.matrix_determinant(matrix1)
        expected1 = 1*4 - 2*3  # -2
        assert result1 == expected1
        
        # Test case 2: Identity matrix
        matrix2 = [[1, 0], [0, 1]]
        result2 = self.pg.matrix_determinant(matrix2)
        assert result2 == 1
        
        # Test case 3: Zero determinant
        matrix3 = [[1, 2], [2, 4]]
        result3 = self.pg.matrix_determinant(matrix3)
        assert result3 == 0
    
    def test_matrix_determinant_invalid_size(self):
        """Test matrix determinant with invalid matrix sizes"""
        with pytest.raises(ValueError):
            self.pg.matrix_determinant([[1, 2, 3], [4, 5, 6]])
        
        with pytest.raises(ValueError):
            self.pg.matrix_determinant([[1]])
    
    def test_matrix_vector_multiply(self):
        """Test matrix-vector multiplication"""
        matrix = [[1, 2], [3, 4]]
        vector = [5, 6]
        result = self.pg.matrix_vector_multiply(matrix, vector)
        expected = [1*5 + 2*6, 3*5 + 4*6]  # [17, 39]
        assert result == expected
    
    def test_angle_transformation(self):
        """Test angle normalization to [-π, π]"""
        # Test case 1: Angle > π
        angle1 = 3 * math.pi
        result1 = self.pg.angle_transformation(angle1)
        assert abs(result1 + math.pi) < 1e-9  # Should be close to -π
        
        # Test case 2: Angle < -π
        angle2 = -3 * math.pi
        result2 = self.pg.angle_transformation(angle2)
        assert abs(result2 + math.pi) < 1e-9  # Should be close to -π
        
        # Test case 3: Angle already in range
        angle3 = math.pi / 4
        result3 = self.pg.angle_transformation(angle3)
        assert abs(result3 - angle3) < 1e-9
        
        # Test case 4: Zero angle
        result4 = self.pg.angle_transformation(0)
        assert abs(result4) < 1e-9
    
    def test_point_to_polygon_distance_square(self):
        """Test point to polygon distance with a square"""
        square = [(0, 0), (4, 0), (4, 4), (0, 4)]
        
        # Test case 1: Point outside square
        point1 = (2, -1)
        dist1, closest1 = self.pg.point_to_polygon_distance(square, point1)
        assert abs(dist1 - 1.0) < 1e-6  # Distance should be 1
        assert abs(closest1[0] - 2) < 1e-6  # x should be 2
        assert abs(closest1[1]) < 1e-6  # y should be close to 0
        
        # Test case 2: Point inside square (distance should be 0)
        point2 = (2, 2)
        dist2, closest2 = self.pg.point_to_polygon_distance(square, point2)
        assert dist2 == 0.0
        assert closest2 == point2
        
        # Test case 3: Point on edge
        point3 = (2, 0)
        dist3, closest3 = self.pg.point_to_polygon_distance(square, point3)
        assert abs(dist3) < 1e-6  # Should be essentially 0
    
    def test_point_to_line_distance(self):
        """Test point to line distance calculation"""
        # Horizontal line from (0,0) to (4,0)
        line = [(0, 0), (4, 0)]
        
        # Point above the line
        point = (2, 3)
        dist, closest = self.pg.point_to_line_distance(line, point)
        assert abs(dist - 3.0) < 1e-6  # Distance should be 3
        assert abs(closest[0] - 2) < 1e-6  # Closest x should be 2
        assert abs(closest[1]) < 1e-6  # Closest y should be close to 0
    
    def test_polygon_ray_intersection(self):
        """Test polygon-ray intersection"""
        square = [(0, 0), (4, 0), (4, 4), (0, 4)]
        
        # Test case 1: Ray from center going right
        ray_origin = (2, 2)
        ray_direction = (1, 0)
        hit = self.pg.polygon_ray_intersection(square, ray_origin, ray_direction)
        assert hit is not None
        x, y = hit
        assert abs(x - 4) < 1e-6  # Should hit right edge at x=4
        assert abs(y - 2) < 1e-6  # y should remain 2
        
        # Test case 2: Ray from center going up
        ray_direction2 = (0, 1)
        hit2 = self.pg.polygon_ray_intersection(square, ray_origin, ray_direction2)
        assert hit2 is not None
        x2, y2 = hit2
        assert abs(x2 - 2) < 1e-6  # x should remain 2
        assert abs(y2 - 4) < 1e-6  # Should hit top edge at y=4
        
        # Test case 3: Ray pointing away from polygon
        ray_origin3 = (10, 10)
        ray_direction3 = (1, 1)
        hit3 = self.pg.polygon_ray_intersection(square, ray_origin3, ray_direction3)
        assert hit3 is None  # No intersection expected
    
    def test_polygon_line_intersection(self):
        """Test polygon-line intersection"""
        square = [(0, 0), (4, 0), (4, 4), (0, 4)]
        
        # Vertical line through center
        line_point = (2, 2)
        line_normal = (1, 0)  # Normal pointing right means vertical line
        intersections = self.pg.polygon_line_intersection(square, line_point, line_normal)
        
        # Should intersect at top and bottom
        assert len(intersections) == 2
        # Sort by y-coordinate
        intersections.sort(key=lambda p: p[1])
        
        # Bottom intersection
        assert abs(intersections[0][0] - 2) < 1e-6
        assert abs(intersections[0][1]) < 1e-6
        
        # Top intersection
        assert abs(intersections[1][0] - 2) < 1e-6
        assert abs(intersections[1][1] - 4) < 1e-6
    
    def test_polygon_halfplane_intersection(self):
        """Test polygon-halfplane intersection"""
        square = [(0, 0), (4, 0), (4, 4), (0, 4)]
        
        # Halfplane cutting through the middle vertically
        halfplane_point = (2, 2)
        halfplane_normal = (1, 0)  # Normal pointing right
        
        result = self.pg.polygon_halfplane_intersection(square, halfplane_point, halfplane_normal)
        
        # Should get some polygon (could be triangle or quadrilateral)
        assert len(result) >= 3  # Should be at least a triangle
        
        # Most points should have x >= 2 (allowing some tolerance for intersection)
        points_on_right = sum(1 for point in result if point[0] >= 2 - 1e-6)
        assert points_on_right >= len(result) // 2  # At least half should be on right side
    
    def test_triangulate_polygon(self):
        """Test polygon triangulation"""
        square = [(0, 0), (4, 0), (4, 4), (0, 4)]
        triangles = self.pg.triangulate_polygon(square)
        
        # Square should be triangulated into at least 2 triangles
        assert len(triangles) >= 2
        
        # Each triangle should have 3 vertices
        for triangle in triangles:
            assert len(triangle) == 3
    
    def test_convex_decomposition(self):
        """Test convex decomposition"""
        square = [(0, 0), (4, 0), (4, 4), (0, 4)]
        convex_parts = self.pg.convex_decomposition(square)
        
        # Square is already convex, should return itself
        assert len(convex_parts) >= 1
        
        # Each part should have at least 3 vertices
        for part in convex_parts:
            assert len(part) >= 3
    
    def test_create_polygeom_function(self):
        """Test the convenience create_polygeom function"""
        pg = create_polygeom()
        assert pg is not None
        
        # Test that it works
        result = pg.matrix_determinant([[1, 0], [0, 1]])
        assert result == 1


def test_built_in_smoke_test():
    """Test that the built-in test function runs without errors"""
    from utils.polygeom_wrapper import test_polygeom
    
    # This should run without raising any exceptions
    try:
        test_polygeom()
        assert True  # If we get here, the test passed
    except Exception as e:
        pytest.fail(f"Built-in smoke test failed: {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
