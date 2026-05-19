#!/usr/bin/env python
import rospy
import math
import heapq
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')

        # RViz Visualization Publishers
        self.pub_expanded = rospy.Publisher('/astar/expanded', GridCells, queue_size=10)
        self.pub_frontier = rospy.Publisher('/astar/frontier', GridCells, queue_size=10)
        self.pub_path = rospy.Publisher('/astar/path', Path, queue_size=10)

        # Map Data Variables
        self.map_data = []
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # Start and Goal Grid Coordinates
        self.start_node = None
        self.goal_node = None

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.start_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        rospy.loginfo("A* Planner Initialized. Waiting for map, start, and goal...")

    # --- CALLBACKS & DATA HANDLING ---

    def map_callback(self, msg):
        #Gets map data from the map_server
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = msg.data
        rospy.loginfo("Map loaded")

    def start_callback(self, msg):
        #Converts RViz '2D Pose Estimate' into grid coordinates
        world_x = msg.pose.pose.position.x
        world_y = msg.pose.pose.position.y
        self.start_node = self.world_to_grid(world_x, world_y)
        rospy.loginfo(f"Start set at grid {self.start_node}")

    def goal_callback(self, msg):
        #Converts RViz '2D Nav Goal' into grid coordinates and triggers A*
        if self.start_node is None:
            rospy.logwarn("Please set a Start Pose first!")
            return
        
        world_x = msg.pose.position.x
        world_y = msg.pose.position.y
        self.goal_node = self.world_to_grid(world_x, world_y)
        rospy.loginfo(f"Goal set at grid {self.goal_node}. Starting A*...")
        
        self.run_astar()

    # --- COORDINATE CONVERSIONS ---

    def world_to_grid(self, x, y):
        #Converts real-world meters into integer grid coordinates
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        #Converts integer grid coordinates back into real-world meters
        world_x = (grid_x * self.map_resolution) + self.map_origin_x + (self.map_resolution / 2.0)
        world_y = (grid_y * self.map_resolution) + self.map_origin_y + (self.map_resolution / 2.0)
        return (world_x, world_y)

    def get_1d_index(self, grid_x, grid_y):
        #Converts 2D (x,y) grid coordinates to the 1D map array index
        return grid_y * self.map_width + grid_x

    # --- A* ALGORITHM LOGIC ---

    def heuristic(self, a, b):
        #Manhattan
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, current):
        #Returns valid neighbors
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] # Up, Right, Down, Left
        
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            
            # Check map bounds
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                idx = self.get_1d_index(nx, ny)
                # Check if obstacle (100) or unknown (-1). Adjust threshold as needed.
                if self.map_data[idx] < 50 and self.map_data[idx] != -1: 
                    neighbors.append((nx, ny))
        return neighbors

    def run_astar(self):
        frontier = []
        heapq.heappush(frontier, (0, self.start_node))
        
        came_from = {}
        g_score = {self.start_node: 0}
        
        expanded_nodes = []
        
        while frontier:
            current = heapq.heappop(frontier)[1]
            expanded_nodes.append(current)

            # Goal Reached!
            if current == self.goal_node:
                self.reconstruct_path(came_from, current)
                return

            for next_node in self.get_neighbors(current):
                new_g = g_score[current] + 1 # Cost to move to neighbor is 1

                if next_node not in g_score or new_g < g_score[next_node]:
                    g_score[next_node] = new_g
                    priority = new_g + self.heuristic(self.goal_node, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

            # Optional: Publish grid cells here for live visualization 
            # (Warning: Publishing every loop can slow down large maps significantly)
            # self.publish_gridcells(self.pub_expanded, expanded_nodes)
            # self.publish_gridcells(self.pub_frontier, [node for _, node in frontier])

        rospy.logwarn("A* Failed: No valid path found to the goal.")

    # --- PATH OPTIMIZATION & PUBLISHING ---

    def reconstruct_path(self, came_from, current):
        raw_path = [current]
        while current in came_from:
            current = came_from[current]
            raw_path.append(current)
        
        raw_path.reverse() # Reverse to get Start -> Goal
        
        optimized_path = self.optimize_path(raw_path)
        
        rospy.loginfo(f"Path found! Raw nodes: {len(raw_path)} | Optimized nodes: {len(optimized_path)}")
        self.publish_path(optimized_path)

    def optimize_path(self, path):
        #Remove redundant nodes using 2D cross product
        if len(path) <= 2:
            return path

        optimized = [path[0]]

        for i in range(1, len(path) - 1):
            p1 = optimized[-1] # The last confirmed point
            p2 = path[i]       # The point we are evaluating
            p3 = path[i + 1]   # The next point

            # Calculate vectors
            dx1 = p2[0] - p1[0]
            dy1 = p2[1] - p1[1]
            dx2 = p3[0] - p2[0]
            dy2 = p3[1] - p2[1]

            # Cross product
            cross_product = (dx1 * dy2) - (dy1 * dx2)

            # If cross product is NOT zero, it's a turn. Keep the node.
            if cross_product != 0:
                optimized.append(p2)

        # Always add the final goal node
        optimized.append(path[-1])
        return optimized

    def publish_path(self, grid_path):
        #Converts grid path to nav_msgs/Path and publishes
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for node in grid_path:
            world_x, world_y = self.grid_to_world(node[0], node[1])
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 # Default neutral orientation
            
            path_msg.poses.append(pose)

        self.pub_path.publish(path_msg)

if __name__ == '__main__':
    try:
        AStarPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass