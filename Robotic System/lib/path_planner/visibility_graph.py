"""
Visibility Graph Path Planner

This module implements a visibility graph-based path planning algorithm for 2D environments
with rectangular obstacles. The visibility graph connects the start and goal points to all
visible vertices of rectangular obstacles, creating an optimal path in terms of Euclidean distance.

The planner accounts for robot dimensions by inflating obstacles (Configuration Space approach).
Each obstacle is expanded by half of the robot's largest dimension on all sides, allowing
the robot to be treated as a point in the inflated space.

Author: Robotic Systems Library
"""

import math
import heapq
import sys
import os
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Import shape classes from utils
sys.path.append("../../")
from lib.utils.shape import create_rectangular_obstacle, Polygon, Point, LineSegment

class VisibilityGraph:
    """
    Visibility Graph Path Planner
    
    Creates a graph where nodes are start/goal points and rectangular obstacle vertices,
    and edges connect nodes that are mutually visible (no obstacles between them).
    The planner accounts for robot dimensions by inflating obstacles.
    """
    
    def __init__(self, obstacles: List[Polygon], robot_width: float = 0.0, robot_length: float = 0.0):
        """
        Initialize visibility graph with rectangular obstacles and robot dimensions.
        
        Args:
            obstacles: List of Polygon objects representing rectangular obstacles
            robot_width: Width of the robot
            robot_length: Length of the robot
        """
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.robot_radius = max(robot_width, robot_length) / 2.0  # Half of the largest dimension
        
        # Inflate obstacles to account for robot dimensions
        self.original_obstacles = obstacles
        self.obstacles = self._inflate_obstacles(obstacles)
        self.vertices = self._extract_vertices()
        self.graph = {}
    
    def _inflate_obstacles(self, obstacles: List[Polygon]) -> List[Polygon]:
        """
        Inflate rectangular obstacles by robot radius to account for robot dimensions.
        This creates the Configuration Space (C-Space) representation where the robot
        can be treated as a point.
        
        Args:
            obstacles: List of original rectangular obstacles
            
        Returns:
            List of inflated rectangular obstacles
        """
        if self.robot_radius <= 0:
            return obstacles
        
        inflated_obstacles = []
        
        for obstacle in obstacles:
            # For rectangular obstacles, we need to find the bounding box
            # and expand it by robot_radius on all sides
            
            # Extract x and y coordinates of all vertices
            x_coords = [vertex.x for vertex in obstacle.vertices]
            y_coords = [vertex.y for vertex in obstacle.vertices]
            
            # Find the bounding box of the original obstacle
            min_x = min(x_coords)
            max_x = max(x_coords)
            min_y = min(y_coords)
            max_y = max(y_coords)
            
            # Calculate original dimensions
            original_width = max_x - min_x
            original_height = max_y - min_y
            
            # Expand the bounding box by robot_radius on all sides
            # New bottom-left corner (shifted by robot_radius)
            inflated_min_x = min_x - self.robot_radius
            inflated_min_y = min_y - self.robot_radius
            
            # Calculate the new inflated dimensions (add 2 * robot_radius)
            inflated_width = original_width + 2 * self.robot_radius
            inflated_height = original_height + 2 * self.robot_radius
            
            # Create the inflated rectangular obstacle using bottom-left corner
            inflated_obstacle = create_rectangular_obstacle(
                inflated_min_x, inflated_min_y, inflated_width, inflated_height
            )
            
            inflated_obstacles.append(inflated_obstacle)
        
        return inflated_obstacles
    
    def _extract_vertices(self) -> List[Point]:
        """Extract all vertices from rectangular obstacles."""
        vertices = []
        for obstacle in self.obstacles:
            vertices.extend(obstacle.vertices)
        return vertices
    
    def _line_intersects_polygon(self, line: LineSegment, polygon: Polygon) -> bool:
        """
        Check if a line segment intersects with any edge of a rectangular polygon.
        
        Args:
            line: Line segment to check
            polygon: Rectangular polygon to check intersection with
            
        Returns:
            True if line intersects polygon, False otherwise
        """
        for edge in polygon.edges:
            if self._segments_intersect(line, edge):
                return True
        return False
    
    def _segments_intersect(self, seg1: LineSegment, seg2: LineSegment) -> bool:
        """
        Check if two line segments intersect using cross product method.
        
        Args:
            seg1: First line segment
            seg2: Second line segment
            
        Returns:
            True if segments intersect, False otherwise
        """
        def ccw(A: Point, B: Point, C: Point) -> bool:
            return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)
        
        A, B = seg1.p1, seg1.p2
        C, D = seg2.p1, seg2.p2
        
        # Check if endpoints are the same (touching is allowed)
        if A == C or A == D or B == C or B == D:
            return False
        
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
    
    def _is_visible(self, p1: Point, p2: Point) -> bool:
        """
        Check if two points are mutually visible (no rectangular obstacles block the line of sight).
        
        Args:
            p1: First point
            p2: Second point
            
        Returns:
            True if points are visible to each other, False otherwise
        """
        if p1 == p2:
            return False
        
        line = LineSegment(p1, p2)
        
        # Check if line intersects any obstacle
        for obstacle in self.obstacles:
            if self._line_intersects_polygon(line, obstacle):
                return False
        
        # Check if line passes through interior of any rectangular obstacle
        midpoint = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)
        for obstacle in self.obstacles:
            if obstacle.contains_point(midpoint):
                return False
        
        return True
    
    def _build_graph(self, start: Point, goal: Point) -> None:
        """
        Build the visibility graph including start and goal points.
        
        Args:
            start: Starting point
            goal: Goal point
        """
        self.graph = {}
        all_points = [start, goal] + self.vertices
        
        for i, point in enumerate(all_points):
            self.graph[point] = []
            for j, other_point in enumerate(all_points):
                if i != j and self._is_visible(point, other_point):
                    distance = point.distance_to(other_point)
                    self.graph[point].append((other_point, distance))
    
    def _dijkstra(self, start: Point, goal: Point) -> Tuple[Optional[List[Point]], float]:
        """
        Find shortest path using Dijkstra's algorithm.
        
        Args:
            start: Starting point
            goal: Goal point
            
        Returns:
            Tuple of (path as list of points, total distance)
            Returns (None, float('inf')) if no path exists
        """
        distances = {point: float('inf') for point in self.graph}
        distances[start] = 0
        previous = {}
        unvisited = [(0, 0, start)]  # Added counter as tie-breaker
        visited = set()
        counter = 1  # Counter to ensure unique ordering
        
        while unvisited:
            current_dist, _, current = heapq.heappop(unvisited)  # Updated to handle 3-tuple
            
            if current in visited:
                continue
            
            visited.add(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current is not None:
                    path.append(current)
                    current = previous.get(current)
                path.reverse()
                return path, distances[goal]
            
            for neighbor, edge_weight in self.graph[current]:
                if neighbor in visited:
                    continue
                
                distance = current_dist + edge_weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current
                    heapq.heappush(unvisited, (distance, counter, neighbor))  # Added counter as tie-breaker
                    counter += 1
        
        return None, float('inf')
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[Optional[List[Tuple[float, float]]], float]:
        """
        Plan a path from start to goal using visibility graph with rectangular obstacles.
        
        Args:
            start: Starting position as (x, y) tuple
            goal: Goal position as (x, y) tuple
            
        Returns:
            Tuple of (path as list of (x, y) tuples, total distance)
            Returns (None, float('inf')) if no path exists
        """
        start_point = Point(start[0], start[1])
        goal_point = Point(goal[0], goal[1])
        
        # Check if start or goal is inside a rectangular obstacle
        for obstacle in self.obstacles:
            if obstacle.contains_point(start_point):
                print(f"Warning: Start point {start} is inside an obstacle")
                return None, float('inf')
            if obstacle.contains_point(goal_point):
                print(f"Warning: Goal point {goal} is inside an obstacle")
                return None, float('inf')
        
        # Build visibility graph
        self._build_graph(start_point, goal_point)
        
        # Find shortest path
        path_points, distance = self._dijkstra(start_point, goal_point)
        
        if path_points is None:
            return None, float('inf')
        
        # Convert back to tuple format
        path = [(point.x, point.y) for point in path_points]
        return path, distance
    
    def visualize_graph(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """
        Visualize the visibility graph with rectangular obstacles using matplotlib.
        Shows both original obstacles and inflated obstacles for robot dimensions.
        
        Args:
            start: Starting position as (x, y) tuple
            goal: Goal position as (x, y) tuple
        """
        
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        
        # Draw original obstacles (actual physical obstacles)
        for i, obstacle in enumerate(self.original_obstacles):
            vertices = [(v.x, v.y) for v in obstacle.vertices]
            # Add label only to the first obstacle
            label = 'Physical Obstacles' if i == 0 else None
            polygon = patches.Polygon(vertices, closed=True, facecolor='red', alpha=0.7, 
                                    edgecolor='darkred', linewidth=2, label=label)
            ax.add_patch(polygon)
        
        # Draw inflated obstacles (configuration space obstacles) if robot has dimensions
        if self.robot_radius > 0:
            for i, obstacle in enumerate(self.obstacles):
                vertices = [(v.x, v.y) for v in obstacle.vertices]
                # Add label only to the first inflated obstacle
                label = 'Inflated Obstacles (C-Space)' if i == 0 else None
                polygon = patches.Polygon(vertices, closed=True, facecolor='orange', alpha=0.3, 
                                        edgecolor='darkorange', linewidth=1, linestyle='--', 
                                        label=label)
                ax.add_patch(polygon)
        
        # Build graph for visualization
        start_point = Point(start[0], start[1])
        goal_point = Point(goal[0], goal[1])
        self._build_graph(start_point, goal_point)
        
        # Draw visibility edges
        for point, neighbors in self.graph.items():
            for neighbor, _ in neighbors:
                ax.plot([point.x, neighbor.x], [point.y, neighbor.y], 'b-', alpha=0.3, linewidth=0.5)
        
        # Draw vertices
        for vertex in self.vertices:
            ax.plot(vertex.x, vertex.y, 'ko', markersize=4)
        
        # Draw start and goal
        ax.plot(start[0], start[1], 'go', markersize=8, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')
        
        # Find and draw path
        path, distance = self.plan_path(start, goal)
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            ax.plot(path_x, path_y, 'g-', linewidth=3, label=f'Path (dist: {distance:.2f})')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        title = 'Visibility Graph Path Planning'
        if self.robot_radius > 0:
            title += f' - Robot: {self.robot_width:.1f}x{self.robot_length:.1f} (radius: {self.robot_radius:.1f})'
        else:
            title += ' - Point Robot'
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        plt.show()


# Example usage
if __name__ == "__main__":
    print("Starting visibility graph example...")
    
    # Create rectangular obstacles
    obstacles = [
        create_rectangular_obstacle(2, 2, 2, 1),
        create_rectangular_obstacle(6, 1, 1, 3)
    ]
    
    print(f"Created {len(obstacles)} rectangular obstacles")
    
    # Define robot dimensions
    robot_width = 0.8   # Robot width
    robot_length = 1.0  # Robot length
    print(f"Robot dimensions: {robot_width} x {robot_length}")
    
    # Create visibility graph planner with robot dimensions
    planner = VisibilityGraph(obstacles, robot_width, robot_length)
    print(f"Created visibility graph planner with robot radius: {planner.robot_radius:.2f}")
    
    # Plan path
    start = (0, 0)
    goal = (8, 8)
    print(f"Planning path from {start} to {goal}")
    
    path, distance = planner.plan_path(start, goal)
    
    if path:
        print(f"Path found with distance: {distance:.2f}")
        print("Path waypoints:")
        for i, point in enumerate(path):
            print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f})")
    else:
        print("No path found!")
    
    # Compare with point robot
    print("\n" + "="*50)
    print("Comparison with point robot:")
    point_planner = VisibilityGraph(obstacles, 0, 0)
    point_path, point_distance = point_planner.plan_path(start, goal)
    
    if point_path:
        print(f"Point robot path distance: {point_distance:.2f}")
        print(f"Difference due to robot size: {distance - point_distance:.2f}")
    
    # Visualize the graph and path
    print("\nGenerating visualization...")
    planner.visualize_graph(start, goal)
    
    print("Example completed!")
