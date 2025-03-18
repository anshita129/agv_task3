import pygame
import heapq
import math
import random

class Planner:
    def is_position_clear(self,map_surface, coordinates, map_width, map_height, buffer_zone=5):
        """
        Verify if a given position maintains a minimum buffer distance from any obstacle.

        Args:
            map_surface: The Pygame surface representing the environment map.
            coordinates: The coordinates to evaluate (x, y).
            map_width: Width of the map.
            map_height: Height of the map.
            buffer_zone: Minimum distance to keep from obstacles (default 5 pixels).

        Returns:
            True if the position is sufficiently distant from obstacles, False otherwise.
        """
        CLEAR_COLOR = (255, 255, 255)  # Safe area color
        OBSTACLE_COLOR = (181, 101, 29)  # Obstacle color

        pos_x, pos_y = int(coordinates[0]), int(coordinates[1])
        if pos_x < 0 or pos_x >= map_width or pos_y < 0 or pos_y >= map_height:
            return False

        try:
            if map_surface.get_at((pos_x, pos_y)) != CLEAR_COLOR:
                return False
        except IndexError:
            return False

        for offset_x in range(-buffer_zone, buffer_zone + 1):
            for offset_y in range(-buffer_zone, buffer_zone + 1):
                test_x, test_y = pos_x + offset_x, pos_y + offset_y

                if (test_x < 0 or test_x >= map_width or 
                    test_y < 0 or test_y >= map_height):
                    continue

                try:
                    test_color = map_surface.get_at((test_x, test_y))
                    if test_color[:3] == OBSTACLE_COLOR:
                        return False
                except IndexError:
                    continue

        return True



    def get_path(self, surface, world_height, world_width, start, goal):
        #converting to tuple and integer
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        
        # Constants
        WHITE = (255, 255, 255)
        SAFETY_BUFFER = 7
        
        # Verify that start and goal are valid positions and not too close to walls
        # Note: We allow start and goal to be close to walls if necessary since they're fixed positions
        try:
            if start[0] < 0 or start[0] >= world_width or start[1] < 0 or start[1] >= world_height:
                print(f"Invalid start position: {start}")
                return []
            if goal[0] < 0 or goal[0] >= world_width or goal[1] < 0 or goal[1] >= world_height:
                print(f"Invalid goal position: {goal}")
                return []
            
        except IndexError:
            print(f"Index error checking start/goal positions: {start}, {goal}")
            return []
        
        
        # Define directions (8-directional movement)
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),(1, 1), (1, -1), (-1, 1), (-1, -1)  
        ]
        
        # Create seen set to keep track of evaluated nodes
        seen = set()
        
        
        pq = []
        count = 0  # Tiebreaker for nodes with same distance
        heapq.heappush(pq, (0, count, start))#distane ,count,node
        
        # Dictionary to store distances from start to each node
        distances = {start: 0}
        
        # Dictionary to store the parent of each node (for path reconstruction)
        came_from = {}
        
        # Set a maximum number of iterations to prevent infinite loops
        max_iterations = world_width * world_height
        iteration = 0
        
        while pq and iteration < max_iterations:
            iteration += 1
            # Get node with smallest distance from start
            current_dist, _, current = heapq.heappop(pq)
            # Skip if already seen
            if current in seen:
                continue
            seen.add(current)
            
            # Check if we've reached the goal
            if current[0] == goal[0] and current[1] == goal[1]:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # Check all neighbors
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip if out of bounds
                if (neighbor[0] < 0 or neighbor[0] >= world_width or
                    neighbor[1] < 0 or neighbor[1] >= world_height):
                    continue
                
                # Skip if already seen
                if neighbor in seen:
                    continue
                
                # Skip if neighbor is not white or too close to a wall
                # (except for goal point which is allowed to be near walls)
                if neighbor != goal:
                    if not self.is_position_clear(surface, neighbor, world_width, world_height, SAFETY_BUFFER):
                        continue
                else:
                    # For the goal, just make sure it's not a wall
                    try:
                        if surface.get_at((int(neighbor[0]), int(neighbor[1]))) != WHITE:
                            continue
                    except IndexError:
                        continue
                
                # Calculate movement cost (diagonal movement costs more)
                movement_cost = 1.0 if dx == 0 or dy == 0 else 1.414
                new_distance = distances[current] + movement_cost
                
                # Update distance if this path is better
                if neighbor not in distances or new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    came_from[neighbor] = current
                    
                    # Add to priority queue
                    count += 1
                    heapq.heappush(pq, (new_distance, count, neighbor))
    def get_meeting_point(self, surface, world_height, world_width, pos1, pos2):
        """
        Finds a suitable meeting point between two agents that is at least 5 pixels
        away from any brown wall.
        
        Args:
            surface: The pygame Surface representing the world map
            world_height: Height of the world
            world_width: Width of the world
            pos1: Position of agent 1 (x, y)
            pos2: Position of agent 2 (x, y)
        
        Returns:
            A (x, y) coordinate for the meeting point
        """
        WHITE = (255, 255, 255)
        SAFETY_BUFFER = 5
        
        # First try the exact midpoint
        midpoint_x = (pos1[0] + pos2[0]) / 2.0
        midpoint_y = (pos1[1] + pos2[1]) / 2.0
        midpoint = (int(midpoint_x), int(midpoint_y))
        
        # Check if midpoint is valid and safe distance from walls
        if self.is_position_clear(surface, midpoint, world_width, world_height, SAFETY_BUFFER):
            return midpoint
        
        # If midpoint isn't suitable, try to find a nearby safe space
        found_points = []
        
        for radius in range(1, 100, 2):  # Try increasingly larger circles
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    # Only check points near the circle (to reduce computation)
                    if abs(dx*dx + dy*dy - radius*radius) > radius:
                        continue
                    
                    check_point = (int(midpoint_x + dx), int(midpoint_y + dy))
                    
                    # Skip if out of bounds
                    if (check_point[0] < 0 or check_point[0] >= world_width or
                        check_point[1] < 0 or check_point[1] >= world_height):
                        continue
                    
                    # Check if this point is safe from walls
                    if self.is_position_clear(surface, check_point, world_width, world_height, SAFETY_BUFFER):
                        # Calculate the "goodness" of this point (how equidistant it is from both agents)
                        dist1 = math.sqrt((check_point[0] - pos1[0])**2 + (check_point[1] - pos1[1])**2)
                        dist2 = math.sqrt((check_point[0] - pos2[0])**2 + (check_point[1] - pos2[1])**2)
                        
                        # Prefer points that are more equidistant
                        diff = abs(dist1 - dist2)
                        
                        # Add to our candidates list
                        found_points.append((diff, check_point))
                        
                        # If we found some points, we can stop the extensive search
                        if len(found_points) >= 10:
                            # Get the best point (lowest difference in distances)
                            found_points.sort(key=lambda x: x[0])
                            return found_points[0][1]
            
            # If we found any valid points in this radius, use the best one
            if found_points:
                found_points.sort(key=lambda x: x[0])
                return found_points[0][1]
        
        # First try the midpoint if it's just free (not a wall)
        try:
            if midpoint[0] >= 0 and midpoint[0] < world_width and midpoint[1] >= 0 and midpoint[1] < world_height:
                if surface.get_at(midpoint) == WHITE:
                    return midpoint
        except IndexError:
            pass
            
        # Try a few points between agents
        for t in [0.5, 0.4, 0.6, 0.3, 0.7, 0.2, 0.8]:
            check_point = (int(pos1[0] + t * (pos2[0] - pos1[0])), 
                        int(pos1[1] + t * (pos2[1] - pos1[1])))
            
            if check_point[0] >= 0 and check_point[0] < world_width and check_point[1] >= 0 and check_point[1] < world_height:
                try:
                    if surface.get_at(check_point) == WHITE:
                        return check_point
                except IndexError:
                    pass
        
        # Last resort: find any valid point near either agent
        for agent_pos in [pos1, pos2]:
            for radius in range(1, 20):
                for _ in range(10):
                    angle = random.uniform(0, 2 * math.pi)
                    dx = int(radius * math.cos(angle))
                    dy = int(radius * math.sin(angle))
                    
                    check_point = (int(agent_pos[0] + dx), int(agent_pos[1] + dy))
                    
                    if check_point[0] >= 0 and check_point[0] < world_width and check_point[1] >= 0 and check_point[1] < world_height:
                        try:
                            if surface.get_at(check_point) == WHITE:
                                return check_point
                        except IndexError:
                            pass
        
        # Ultimate fallback: just return position of agent 1
        print("Warning: Could not find any suitable meeting point. Using agent 1's position.")
        return pos1