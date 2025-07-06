"""
Shape utilities for rectangular obstacles

This module provides classes for representing rectangular geometric shapes
used as obstacles in path planning algorithms.

Author: Robotic Systems Library
"""

import math
from typing import List, Tuple


class Point:
    """Represents a 2D point with x, y coordinates."""
    
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return abs(self.x - other.x) < 1e-9 and abs(self.y - other.y) < 1e-9
    
    def __hash__(self):
        return hash((round(self.x, 9), round(self.y, 9)))
    
    def __repr__(self):
        return f"Point({self.x:.3f}, {self.y:.3f})"
    
    def distance_to(self, other: 'Point') -> float:
        """Calculate Euclidean distance to another point."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


class LineSegment:
    """Represents a line segment between two points."""
    
    def __init__(self, p1: Point, p2: Point):
        self.p1 = p1
        self.p2 = p2
    
    def __repr__(self):
        return f"LineSegment({self.p1}, {self.p2})"


class Polygon:
    """Represents a rectangular obstacle."""
    
    def __init__(self, vertices: List[Tuple[float, float]]):
        """
        Initialize rectangular polygon with list of vertices.
        
        Args:
            vertices: List of (x, y) tuples representing rectangle vertices (must be 4 vertices)
        """
        if len(vertices) != 4:
            raise ValueError("Only rectangular obstacles with 4 vertices are supported")
        
        self.vertices = [Point(x, y) for x, y in vertices]
        self.edges = self._create_edges()
    
    def _create_edges(self) -> List[LineSegment]:
        """Create edges from consecutive vertices."""
        edges = []
        for i in range(len(self.vertices)):
            p1 = self.vertices[i]
            p2 = self.vertices[(i + 1) % len(self.vertices)]
            edges.append(LineSegment(p1, p2))
        return edges
    
    def contains_point(self, point: Point) -> bool:
        """
        Check if a point is inside the rectangle.
        
        Args:
            point: Point to check
            
        Returns:
            True if point is inside rectangle, False otherwise
        """
        return self._point_in_rectangle(point)
    
    def _point_in_rectangle(self, point: Point) -> bool:
        """Fast rectangle containment check using bounding box."""
        # Get min/max coordinates
        x_coords = [v.x for v in self.vertices]
        y_coords = [v.y for v in self.vertices]
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        return min_x <= point.x <= max_x and min_y <= point.y <= max_y


def create_rectangular_obstacle(x: float, y: float, width: float, height: float) -> Polygon:
    """
    Create a rectangular obstacle.
    
    Args:
        x: Bottom-left x coordinate
        y: Bottom-left y coordinate
        width: Width of rectangle
        height: Height of rectangle
        
    Returns:
        Polygon object representing the rectangle
    """
    vertices = [
        (x, y),
        (x + width, y),
        (x + width, y + height),
        (x, y + height)
    ]
    return Polygon(vertices)


