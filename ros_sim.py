# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from std_msgs.msg import String
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point
# import json
# import random
# import math

# # Simulation Parameters
# WAREHOUSE_SIZE = 20.0
# SENSING_RANGE = 10.0
# NOISE_STDDEV = 0.5
# OBSERVATION_INTERVAL = 1.0    # Update rate (seconds)
# COMMUNICATION_INTERVAL = 5.0  # Communication rate (seconds)

# # Simulation Parameters
# OFFICE_WIDTH = 20.0  # meters, based on the 20089mm width shown
# OFFICE_LENGTH = 15.0  # meters, approximate based on map scale
# OFFICE_ORIGIN_X = -10.0  # Center the coordinate system
# OFFICE_ORIGIN_Y = -7.5

# # Initial positions for dynamic obstacles (matching office layout)
# DYNAMIC_OBSTACLES = [
#     {'id': 'human_1', 'true_position': [-5.0, 0.0]},  # Near pantry
#     {'id': 'human_2', 'true_position': [5.0, -2.0]}   # Near hardware_2
# ]

# # Initial robot positions (on the orange pathways)
# ROBOT_INIT_POSITIONS = [
#     [-8.0, -5.0],  # Near coe
#     [0.0, 0.0],    # Near pantry
#     [8.0, -3.0]    # Near hardware_2
# ]

# def update_dynamic_obstacles():
#     for obs in DYNAMIC_OBSTACLES:
#         x, y = obs['true_position']
#         dx = random.uniform(-0.2, 0.2)
#         dy = random.uniform(-0.2, 0.2)
#         new_x = min(max(x + dx, 0.0), WAREHOUSE_SIZE)
#         new_y = min(max(y + dy, 0.0), WAREHOUSE_SIZE)
#         obs['true_position'] = [new_x, new_y]

# def simulate_sensor_measurement(robot_pos, obstacle_pos):
#     dx = robot_pos[0] - obstacle_pos[0]
#     dy = robot_pos[1] - obstacle_pos[1]
#     distance = math.hypot(dx, dy)
#     if distance > SENSING_RANGE:
#         return None
#     noisy_x = obstacle_pos[0] + random.gauss(0.0, NOISE_STDDEV)
#     noisy_y = obstacle_pos[1] + random.gauss(0.0, NOISE_STDDEV)
#     return (noisy_x, noisy_y, distance)

# class RobotNode(Node):
#     def __init__(self, robot_id: int):
#         super().__init__(f'robot_{robot_id}')
#         self.robot_id = robot_id
#         self.state = {
#             'position': ROBOT_INIT_POSITIONS[robot_id - 1]
#         }
#         self.beliefs = {}
        
#         # Publishers
#         self.obs_pub = self.create_publisher(String, 'robot_observations', 10)
#         self.viz_pub = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        
#         # Subscribers
#         self.shared_sub = self.create_subscription(
#             String, 'shared_observations', self.shared_callback, 10)
        
#         # Timer for updates
#         self.timer = self.create_timer(OBSERVATION_INTERVAL, self.timer_callback)

#     def simulate_movement(self):
#         """Constrain movement to the office pathways"""
#         pos = self.state['position']
#         delta = [random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5)]
#         new_x = min(max(pos[0] + delta[0], OFFICE_ORIGIN_X), OFFICE_ORIGIN_X + OFFICE_WIDTH)
#         new_y = min(max(pos[1] + delta[1], OFFICE_ORIGIN_Y), OFFICE_ORIGIN_Y + OFFICE_LENGTH)
#         self.state['position'] = [new_x, new_y]

#     def timer_callback(self):
#         self.simulate_movement()
#         observations = []
        
#         for obs in DYNAMIC_OBSTACLES:
#             measurement = simulate_sensor_measurement(
#                 self.state['position'], obs['true_position'])
#             if measurement:
#                 observation = {
#                     'robot_id': self.robot_id,
#                     'obstacle_id': obs['id'],
#                     'measured_position': {
#                         'x': measurement[0],
#                         'y': measurement[1]
#                     },
#                     'distance': measurement[2],
#                     'timestamp': self.get_clock().now().nanoseconds
#                 }
#                 self.beliefs[obs['id']] = observation['measured_position']
#                 observations.append(observation)
        
#         msg_dict = {
#             'robot_id': self.robot_id,
#             'state': {
#                 'position': {
#                     'x': self.state['position'][0],
#                     'y': self.state['position'][1]
#                 }
#             },
#             'observations': observations,
#             'beliefs': self.beliefs
#         }
#         msg = String()
#         msg.data = json.dumps(msg_dict)
#         self.obs_pub.publish(msg)
        
#         # Update visualizations
#         self.publish_visualizations()

#     def shared_callback(self, msg):
#         try:
#             data = json.loads(msg.data)
#             if data.get('to_robot') == self.robot_id:
#                 obs = data.get('observation')
#                 if obs:
#                     self.beliefs[obs['obstacle_id']] = obs['measured_position']
#                     self.get_logger().info(
#                         f'Robot {self.robot_id} received observation of '
#                         f'obstacle {obs["obstacle_id"]} from robot {data["from_robot"]}')
#         except Exception as e:
#             self.get_logger().error(f'Error processing shared observation: {e}')

#     def publish_visualizations(self):
#         marker_array = MarkerArray()
        
#         # Robot marker
#         robot = Marker()
#         robot.header.frame_id = "map"
#         robot.header.stamp = self.get_clock().now().to_msg()
#         robot.ns = f"robot_{self.robot_id}"
#         robot.id = self.robot_id
#         robot.type = Marker.CYLINDER
#         robot.action = Marker.ADD
        
#         robot.pose.position.x = self.state['position'][0]
#         robot.pose.position.y = self.state['position'][1]
#         robot.pose.position.z = 0.0
        
#         robot.scale.x = 0.5
#         robot.scale.y = 0.5
#         robot.scale.z = 0.5
        
#         robot.color.r = 1.0 if self.robot_id == 1 else 0.0
#         robot.color.g = 1.0 if self.robot_id == 2 else 0.0
#         robot.color.b = 1.0 if self.robot_id == 3 else 0.0
#         robot.color.a = 1.0
        
#         marker_array.markers.append(robot)
        
#         # Belief markers
#         for obs_id, belief in self.beliefs.items():
#             belief_marker = Marker()
#             belief_marker.header.frame_id = "map"
#             belief_marker.header.stamp = self.get_clock().now().to_msg()
#             belief_marker.ns = f"belief_{self.robot_id}"
#             belief_marker.id = hash(obs_id) % 10000
#             belief_marker.type = Marker.SPHERE
#             belief_marker.action = Marker.ADD
            
#             belief_marker.pose.position.x = belief['x']
#             belief_marker.pose.position.y = belief['y']
#             belief_marker.pose.position.z = 0.3
            
#             belief_marker.scale.x = 0.3
#             belief_marker.scale.y = 0.3
#             belief_marker.scale.z = 0.3
            
#             belief_marker.color.r = 1.0 if self.robot_id == 1 else 0.0
#             belief_marker.color.g = 1.0 if self.robot_id == 2 else 0.0
#             belief_marker.color.b = 1.0 if self.robot_id == 3 else 0.0
#             belief_marker.color.a = 0.5
            
#             marker_array.markers.append(belief_marker)
        
#         # True obstacle positions
#         for i, obs in enumerate(DYNAMIC_OBSTACLES):
#             obstacle_marker = Marker()
#             obstacle_marker.header.frame_id = "map"
#             obstacle_marker.header.stamp = self.get_clock().now().to_msg()
#             obstacle_marker.ns = "true_obstacles"
#             obstacle_marker.id = i
#             obstacle_marker.type = Marker.SPHERE
#             obstacle_marker.action = Marker.ADD
            
#             obstacle_marker.pose.position.x = obs['true_position'][0]
#             obstacle_marker.pose.position.y = obs['true_position'][1]
#             obstacle_marker.pose.position.z = 0.3
            
#             obstacle_marker.scale.x = 0.4
#             obstacle_marker.scale.y = 0.4
#             obstacle_marker.scale.z = 0.4
            
#             obstacle_marker.color.r = 1.0
#             obstacle_marker.color.g = 0.0
#             obstacle_marker.color.b = 0.0
#             obstacle_marker.color.a = 1.0
            
#             marker_array.markers.append(obstacle_marker)
        
#         self.viz_pub.publish(marker_array)

# class CentralObservationManager(Node):
#     def __init__(self):
#         super().__init__('central_observation_manager')
#         self.robot_data = {}
#         self.data_sub = self.create_subscription(
#             String, 'robot_observations', self.data_callback, 10)
#         self.share_pub = self.create_publisher(
#             String, 'shared_observations', 10)
#         self.viz_pub = self.create_publisher(
#             MarkerArray, 'communication_visualization', 10)
#         self.comm_timer = self.create_timer(
#             COMMUNICATION_INTERVAL, self.comm_callback)

#     def data_callback(self, msg):
#         try:
#             data = json.loads(msg.data)
#             robot_id = data.get('robot_id')
#             if robot_id is not None:
#                 self.robot_data[str(robot_id)] = data
#         except Exception as e:
#             self.get_logger().error(f'Error parsing robot data: {e}')

#     def comm_callback(self):
#         self.get_logger().info("\n=== Communication Cycle Start ===")
        
#         # Visualization for communication events
#         comm_markers = MarkerArray()
#         marker_id = 0
        
#         for rec_id, data in self.robot_data.items():
#             for obs in data.get('observations', []):
#                 obs_id = obs.get('obstacle_id')
#                 rec_distance = obs.get('distance', float('inf'))
                
#                 if rec_distance > 5.0:
#                     for send_id, other_data in self.robot_data.items():
#                         if send_id == rec_id:
#                             continue
#                         for other_obs in other_data.get('observations', []):
#                             if (other_obs.get('obstacle_id') == obs_id and
#                                 other_obs.get('distance', float('inf')) < rec_distance):
                                
#                                 # Share observation
#                                 share_msg = {
#                                     'from_robot': send_id,
#                                     'to_robot': rec_id,
#                                     'observation': other_obs
#                                 }
#                                 msg = String()
#                                 msg.data = json.dumps(share_msg)
#                                 self.share_pub.publish(msg)
                                
#                                 # Log communication event
#                                 self.get_logger().info(
#                                     f'Sharing: Robot {send_id} -> Robot {rec_id} '
#                                     f'(obstacle {obs_id})')
                                
#                                 # Visualize communication
#                                 line_marker = Marker()
#                                 line_marker.header.frame_id = "map"
#                                 line_marker.header.stamp = self.get_clock().now().to_msg()
#                                 line_marker.ns = "communication"
#                                 line_marker.id = marker_id
#                                 line_marker.type = Marker.ARROW
#                                 line_marker.action = Marker.ADD
                                
#                                 # Get positions of sending and receiving robots
#                                 send_pos = other_data['state']['position']
#                                 rec_pos = data['state']['position']
                                
#                                 start = Point()
#                                 start.x = send_pos['x']
#                                 start.y = send_pos['y']
#                                 start.z = 0.5
                                
#                                 end = Point()
#                                 end.x = rec_pos['x']
#                                 end.y = rec_pos['y']
#                                 end.z = 0.5
                                
#                                 line_marker.points = [start, end]
                                
#                                 line_marker.scale.x = 0.1  # shaft diameter
#                                 line_marker.scale.y = 0.2  # head diameter
#                                 line_marker.scale.z = 0.2  # head length
                                
#                                 line_marker.color.r = 1.0
#                                 line_marker.color.g = 1.0
#                                 line_marker.color.b = 0.0
#                                 line_marker.color.a = 0.8
                                
#                                 # Make the communication visualization temporary
#                                 line_marker.lifetime.sec = 2
                                
#                                 comm_markers.markers.append(line_marker)
#                                 marker_id += 1
#                                 break
        
#         # Publish communication visualization
#         if comm_markers.markers:
#             self.viz_pub.publish(comm_markers)
        
#         self.get_logger().info("=== Communication Cycle End ===\n")
#         self.robot_data.clear()

# def main(args=None):
#     rclpy.init(args=args)
    
#     num_robots = 3
#     robot_nodes = [RobotNode(i+1) for i in range(num_robots)]
#     central_node = CentralObservationManager()
    
#     executor = MultiThreadedExecutor()
#     for node in robot_nodes:
#         executor.add_node(node)
#     executor.add_node(central_node)
    
#     try:
#         while rclpy.ok():
#             update_dynamic_obstacles()
#             executor.spin_once(timeout_sec=0.1)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         for node in robot_nodes:
#             node.destroy_node()
#         central_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
"""
Multi-Robot Warehouse Simulation

Features:
  • Custom warehouse environment (grid with shelves as obstacles)
  • Each robot receives a random goal; an A*-based planner computes a path.
  • Naïve collision avoidance: robots stop if a dynamic obstacle gets too close.
  • Dynamic obstacles (simulated humans) are updated as in RMF CrowdSim.
  • Robots sense dynamic obstacles realistically (with noise and range limits).
  • A centralized observation sharing infrastructure collects robot data
    and runs a modular (by default, naïve) observation sharing strategy periodically.
  • Performance is measured by task completion times and total communication bandwidth.
  • Visualization markers (robots, planned paths, beliefs, communication arrows) are published for RViz.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import random
import math
import time
from heapq import heappush, heappop

# -----------------------------------------------------------------------------
# Environment & Simulation Parameters
# -----------------------------------------------------------------------------

# Warehouse dimensions (meters) and coordinate origin
WAREHOUSE_WIDTH      = 50.0
WAREHOUSE_LENGTH     = 30.0
WAREHOUSE_ORIGIN_X   = -25.0
WAREHOUSE_ORIGIN_Y   = -15.0

CELL_SIZE = 1.0  # each grid cell is 1m x 1m

SENSING_RANGE = 10.0          # max sensor range in meters
NOISE_STDDEV  = 0.5           # sensor noise standard deviation
OBSERVATION_INTERVAL = 1.0    # update rate [s] for movement and sensing
COMMUNICATION_INTERVAL = 5.0  # observation sharing rate [s]

# -----------------------------------------------------------------------------
# Grid Environment & Shelving (static obstacles)
# -----------------------------------------------------------------------------

# Compute grid dimensions (number of cells in rows/cols)
GRID_COLS = int(WAREHOUSE_WIDTH / CELL_SIZE)
GRID_ROWS = int(WAREHOUSE_LENGTH / CELL_SIZE)

# Create a grid (0 = free; 1 = shelf/obstacle)
grid = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
def create_shelves(grid):
    """
    Create shelves: for example, block out region in rows 10 to 19
    every fourth column to simulate aisles.
    """
    for i in range(GRID_ROWS):
        for j in range(GRID_COLS):
            if 10 <= i < 20 and (j % 4 == 2 or j % 4 == 3):
                grid[i][j] = 1
create_shelves(grid)

# Predefined goal positions for tasks (world coordinates)
GOAL_POSITIONS = [
    (-20, 10),
    (0, 14),
    (15, -5),
    (-5, -10),
    (10, 0)
]

# -----------------------------------------------------------------------------
# Dynamic Obstacles (Simulated Humans – later replace with RMF CrowdSim)
# -----------------------------------------------------------------------------

# Each dynamic obstacle initially is placed within warehouse bounds.
DYNAMIC_OBSTACLES = [
    {'id': 'human_1', 'true_position': [-15.0, 5.0]},
    {'id': 'human_2', 'true_position': [10.0, -10.0]}
]

def update_dynamic_obstacles():
    """Update positions of dynamic obstacles with small random drift."""
    for obs in DYNAMIC_OBSTACLES:
        x, y = obs["true_position"]
        dx = random.uniform(-0.3, 0.3)
        dy = random.uniform(-0.3, 0.3)
        new_x = min(max(x + dx, WAREHOUSE_ORIGIN_X), WAREHOUSE_ORIGIN_X + WAREHOUSE_WIDTH)
        new_y = min(max(y + dy, WAREHOUSE_ORIGIN_Y), WAREHOUSE_ORIGIN_Y + WAREHOUSE_LENGTH)
        obs["true_position"] = [new_x, new_y]

# -----------------------------------------------------------------------------
# Utility: Converting between world coordinates and grid cells
# -----------------------------------------------------------------------------

def world_to_grid(pos):
    """Convert a world coordinate (x, y) to grid cell (row, col)."""
    col = int((pos[0] - WAREHOUSE_ORIGIN_X) / CELL_SIZE)
    row = int((pos[1] - WAREHOUSE_ORIGIN_Y) / CELL_SIZE)
    return (row, col)

def grid_to_world(cell):
    """Convert a grid cell (row, col) to world coordinate (x, y) at cell center."""
    x = WAREHOUSE_ORIGIN_X + (cell[1] + 0.5) * CELL_SIZE
    y = WAREHOUSE_ORIGIN_Y + (cell[0] + 0.5) * CELL_SIZE
    return (x, y)

# -----------------------------------------------------------------------------
# A* Grid-Based Path Planner
# -----------------------------------------------------------------------------

def astar(grid, start, goal):
    """Compute a path from start to goal using A* on the grid.
       start and goal are (row, col) tuples.
       Returns a list of grid cells (row, col) from start to goal, or None."""
    ROWS = len(grid)
    COLS = len(grid[0])
    # 4-connected grid
    neighbors = [(0,1), (1,0), (0,-1), (-1,0)]

    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    
    open_set = {start}
    came_from = {}
    g_score = { (r,c): float('inf') for r in range(ROWS) for c in range(COLS) }
    g_score[start] = 0
    f_score = { (r,c): float('inf') for r in range(ROWS) for c in range(COLS) }
    f_score[start] = heuristic(start, goal)
    open_heap = []
    heappush(open_heap, (f_score[start], start))
    
    while open_heap:
        current = heappop(open_heap)[1]
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return list(reversed(path))
        open_set.discard(current)
        for d in neighbors:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if 0 <= neighbor[0] < ROWS and 0 <= neighbor[1] < COLS:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue  # obstacle cell
                tentative_g = g_score[current] + 1
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        heappush(open_heap, (f_score[neighbor], neighbor))
    return None

# -----------------------------------------------------------------------------
# Observation Sharing Strategy (Modular)
# -----------------------------------------------------------------------------

class ObservationSharingStrategy:
    def select_observations(self, robot_data):
        """Given a dict of robot data (robot_id -> data), return a list of share messages."""
        return []

class NaiveObservationSharing(ObservationSharingStrategy):
    def __init__(self, threshold=2.0):
        self.threshold = threshold
    def select_observations(self, robot_data):
        share_msgs = []
        # For each robot, check if any other has a significantly better observation
        for rec_id, data in robot_data.items():
            for obs in data.get("observations", []):
                obs_id = obs.get("obstacle_id")
                rec_dist = obs.get("distance", float('inf'))
                for send_id, other_data in robot_data.items():
                    if send_id == rec_id:
                        continue
                    for other_obs in other_data.get("observations", []):
                        if other_obs.get("obstacle_id") == obs_id:
                            # If other robot sees the obstacle significantly closer, share it.
                            if other_obs.get("distance", float('inf')) + self.threshold < rec_dist:
                                share_msgs.append({
                                    "from_robot": send_id,
                                    "to_robot": rec_id,
                                    "observation": other_obs
                                })
        return share_msgs

# Use the naive strategy by default
observation_sharing_strategy = NaiveObservationSharing()

# Global bandwidth measurement (bytes transmitted)
global_bandwidth_usage = 0

# -----------------------------------------------------------------------------
# Robot Node
# -----------------------------------------------------------------------------

class RobotNode(Node):
    def __init__(self, robot_id: int):
        super().__init__(f"robot_{robot_id}")
        self.robot_id = robot_id

        # Assign a random goal from the available list.
        self.goal_world = random.choice(GOAL_POSITIONS)
        self.goal_reached = False
        self.start_time = time.time()
        self.completion_time = None

        # Pick an initial free position (using grid sampling)
        while True:
            x = random.uniform(WAREHOUSE_ORIGIN_X, WAREHOUSE_ORIGIN_X + WAREHOUSE_WIDTH)
            y = random.uniform(WAREHOUSE_ORIGIN_Y, WAREHOUSE_ORIGIN_Y + WAREHOUSE_LENGTH)
            cell = world_to_grid((x, y))
            if grid[cell[0]][cell[1]] == 0:
                self.state = {"position": [x, y]}
                break

        self.current_cell = world_to_grid(self.state["position"])
        self.goal_cell = world_to_grid(self.goal_world)
        self.path = astar(grid, self.current_cell, self.goal_cell)
        if self.path is None:
            self.get_logger().error("No valid path found!")
            self.world_path = []
        else:
            self.world_path = [grid_to_world(cell) for cell in self.path]
        self.path_index = 0

        # Beliefs: each robot stores its (noisy) observations of dynamic obstacles.
        self.beliefs = {}
        
        # ROS publishers/subscribers
        self.obs_pub = self.create_publisher(String, "robot_observations", 10)
        self.shared_sub = self.create_subscription(String, "shared_observations", self.shared_callback, 10)
        self.viz_pub = self.create_publisher(MarkerArray, "visualization_markers", 10)
        
        self.timer = self.create_timer(OBSERVATION_INTERVAL, self.timer_callback)

    def simulate_movement(self):
        """
        Move along the planned path using a naïve grid-based method.
        Also, if a dynamic obstacle is too close, robot stops temporarily.
        """
        if self.goal_reached:
            return

        # Check collision: if any dynamic obstacle is too close, pause.
        pos = self.state["position"]
        for obs in DYNAMIC_OBSTACLES:
            dx = pos[0] - obs["true_position"][0]
            dy = pos[1] - obs["true_position"][1]
            if math.hypot(dx, dy) < 1.0:
                self.get_logger().info(f"Robot {self.robot_id} pausing for obstacle {obs['id']}")
                return  # Do not advance along the path this cycle

        # Move toward current target cell in world_path.
        if self.path_index < len(self.world_path):
            target = self.world_path[self.path_index]
            current = self.state["position"]
            dx = target[0] - current[0]
            dy = target[1] - current[1]
            dist = math.hypot(dx, dy)
            step = 0.5  # step size per update (can adjust for realism)
            if dist < step:
                # Reached current waypoint; move to next
                self.state["position"] = target
                self.path_index += 1
                if self.path_index >= len(self.world_path):
                    self.goal_reached = True
                    self.completion_time = time.time() - self.start_time
                    self.get_logger().info(
                        f"Robot {self.robot_id} reached goal in {self.completion_time:.2f} seconds."
                    )
            else:
                # Advance along the vector toward target
                ratio = step / dist
                new_x = current[0] + dx * ratio
                new_y = current[1] + dy * ratio
                self.state["position"] = [new_x, new_y]

    def timer_callback(self):
        """Called at OBSERVATION_INTERVAL: update movement, sense obstacles, and broadcast state."""
        self.simulate_movement()

        observations = []
        # Measure dynamic obstacles using a simulated noisy sensor
        for obs in DYNAMIC_OBSTACLES:
            measurement = simulate_sensor_measurement(
                self.state["position"], obs["true_position"]
            )
            if measurement:
                observation = {
                    "robot_id": self.robot_id,
                    "obstacle_id": obs["id"],
                    "measured_position": {"x": measurement[0], "y": measurement[1]},
                    "distance": measurement[2],
                    "timestamp": self.get_clock().now().nanoseconds
                }
                self.beliefs[obs["id"]] = observation["measured_position"]
                observations.append(observation)

        # Package state information
        msg_dict = {
            "robot_id": self.robot_id,
            "state": {"position": {"x": self.state["position"][0], "y": self.state["position"][1]}},
            "observations": observations,
            "beliefs": self.beliefs
        }
        msg = String()
        msg.data = json.dumps(msg_dict)

        # Publish and measure bandwidth usage
        global global_bandwidth_usage
        data_bytes = len(msg.data.encode("utf-8"))
        global_bandwidth_usage += data_bytes
        self.obs_pub.publish(msg)
        self.get_logger().debug(f"Robot {self.robot_id} published {data_bytes} bytes")

        # Publish visualization markers
        self.publish_visualizations()

    def shared_callback(self, msg):
        """Receive shared observation messages from the central manager."""
        try:
            data = json.loads(msg.data)
            if data.get("to_robot") == self.robot_id:
                obs = data.get("observation")
                if obs:
                    self.beliefs[obs["obstacle_id"]] = obs["measured_position"]
                    self.get_logger().info(
                        f"Robot {self.robot_id} updated belief for obstacle {obs['obstacle_id']} "
                        f"from robot {data['from_robot']}"
                    )
        except Exception as e:
            self.get_logger().error(f"Robot {self.robot_id} error in shared_callback: {e}")

    def publish_visualizations(self):
        """Publish markers for robot, planned path, current beliefs, and true dynamic obstacles."""
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()

        # Robot marker (cylinder)
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = timestamp
        robot_marker.ns = f"robot_{self.robot_id}"
        robot_marker.id = self.robot_id
        robot_marker.type = Marker.CYLINDER
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.state["position"][0]
        robot_marker.pose.position.y = self.state["position"][1]
        robot_marker.pose.position.z = 0.25
        robot_marker.scale.x = 0.5
        robot_marker.scale.y = 0.5
        robot_marker.scale.z = 0.5
        if self.robot_id == 1:
            robot_marker.color.r = 1.0; robot_marker.color.g = 0.0; robot_marker.color.b = 0.0
        elif self.robot_id == 2:
            robot_marker.color.r = 0.0; robot_marker.color.g = 1.0; robot_marker.color.b = 0.0
        else:
            robot_marker.color.r = 0.0; robot_marker.color.g = 0.0; robot_marker.color.b = 1.0
        robot_marker.color.a = 1.0
        marker_array.markers.append(robot_marker)

        # Planned path marker (line strip)
        if self.world_path:
            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = timestamp
            path_marker.ns = f"path_{self.robot_id}"
            path_marker.id = self.robot_id + 1000
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.2
            # Use same color as robot
            path_marker.color = robot_marker.color
            path_marker.color.a = 0.8
            for point in self.world_path:
                pt = Point()
                pt.x = point[0]
                pt.y = point[1]
                pt.z = 0.1
                path_marker.points.append(pt)
            marker_array.markers.append(path_marker)

        # Belief markers (spheres, semi-transparent)
        for obs_id, belief in self.beliefs.items():
            belief_marker = Marker()
            belief_marker.header.frame_id = "map"
            belief_marker.header.stamp = timestamp
            belief_marker.ns = f"belief_{self.robot_id}"
            # Ensure unique marker id using hash
            belief_marker.id = abs(hash(obs_id)) % 10000
            belief_marker.type = Marker.SPHERE
            belief_marker.action = Marker.ADD
            belief_marker.pose.position.x = belief["x"]
            belief_marker.pose.position.y = belief["y"]
            belief_marker.pose.position.z = 0.3
            belief_marker.scale.x = 0.3
            belief_marker.scale.y = 0.3
            belief_marker.scale.z = 0.3
            belief_marker.color.r = robot_marker.color.r
            belief_marker.color.g = robot_marker.color.g
            belief_marker.color.b = robot_marker.color.b
            belief_marker.color.a = 0.5
            marker_array.markers.append(belief_marker)

        # True obstacle positions (red spheres)
        for i, obs in enumerate(DYNAMIC_OBSTACLES):
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = "map"
            obstacle_marker.header.stamp = timestamp
            obstacle_marker.ns = "true_obstacles"
            # Use a unique id combining index and robot id
            obstacle_marker.id = i + self.robot_id * 10000
            obstacle_marker.type = Marker.SPHERE
            obstacle_marker.action = Marker.ADD
            obstacle_marker.pose.position.x = obs["true_position"][0]
            obstacle_marker.pose.position.y = obs["true_position"][1]
            obstacle_marker.pose.position.z = 0.3
            obstacle_marker.scale.x = 0.4
            obstacle_marker.scale.y = 0.4
            obstacle_marker.scale.z = 0.4
            obstacle_marker.color.r = 1.0
            obstacle_marker.color.g = 0.0
            obstacle_marker.color.b = 0.0
            obstacle_marker.color.a = 1.0
            marker_array.markers.append(obstacle_marker)

        self.viz_pub.publish(marker_array)

# -----------------------------------------------------------------------------
# Centralized Infrastructure for Observation Sharing
# -----------------------------------------------------------------------------

class CentralObservationManager(Node):
    def __init__(self):
        super().__init__("central_observation_manager")
        self.robot_data = {}  # dictionary keyed by robot_id (as string)
        self.data_sub = self.create_subscription(String, "robot_observations", self.data_callback, 10)
        self.share_pub = self.create_publisher(String, "shared_observations", 10)
        self.viz_pub = self.create_publisher(MarkerArray, "communication_visualization", 10)
        self.comm_timer = self.create_timer(COMMUNICATION_INTERVAL, self.comm_callback)

    def data_callback(self, msg):
        try:
            data = json.loads(msg.data)
            robot_id = data.get("robot_id")
            if robot_id is not None:
                self.robot_data[str(robot_id)] = data
        except Exception as e:
            self.get_logger().error(f"Central manager error parsing robot data: {e}")

    def comm_callback(self):
        self.get_logger().info("=== Communication Cycle Start ===")
        global global_bandwidth_usage
        comm_msgs = observation_sharing_strategy.select_observations(self.robot_data)
        marker_array = MarkerArray()
        marker_id = 0
        for share in comm_msgs:
            # Publish each share message
            msg = String()
            msg.data = json.dumps(share)
            data_bytes = len(msg.data.encode("utf-8"))
            global_bandwidth_usage += data_bytes
            self.share_pub.publish(msg)
            self.get_logger().info(
                f"Sharing observation: Robot {share['from_robot']} -> Robot {share['to_robot']} (obstacle {share['observation']['obstacle_id']}); {data_bytes} bytes.")

            # Also create a visualization arrow from sender to receiver.
            sender_data = self.robot_data.get(str(share["from_robot"]))
            rec_data = self.robot_data.get(str(share["to_robot"]))
            if sender_data is None or rec_data is None:
                continue
            start_pt = Point(x=sender_data["state"]["position"]["x"],
                             y=sender_data["state"]["position"]["y"], z=0.5)
            end_pt = Point(x=rec_data["state"]["position"]["x"],
                           y=rec_data["state"]["position"]["y"], z=0.5)
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "communication"
            arrow.id = marker_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [start_pt, end_pt]
            arrow.scale.x = 0.1  # shaft diameter
            arrow.scale.y = 0.2  # head diameter
            arrow.scale.z = 0.2  # head length
            arrow.color.r = 1.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 0.8
            # Arrow persists only briefly so that communication is highlighted.
            arrow.lifetime.sec = 2
            marker_array.markers.append(arrow)
            marker_id += 1

        if marker_array.markers:
            self.viz_pub.publish(marker_array)
        self.get_logger().info("=== Communication Cycle End ===")
        self.get_logger().info(f"Total bandwidth usage (so far): {global_bandwidth_usage} bytes")
        self.robot_data.clear()

# -----------------------------------------------------------------------------
# Main Execution
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    # Create nodes: several robots and the central observation manager.
    num_robots = 3
    robot_nodes = [RobotNode(i+1) for i in range(num_robots)]
    central_node = CentralObservationManager()

    executor = MultiThreadedExecutor()
    for node in robot_nodes:
        executor.add_node(node)
    executor.add_node(central_node)

    try:
        while rclpy.ok():
            update_dynamic_obstacles()  # Update dynamic obstacles every cycle
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for node in robot_nodes:
            node.destroy_node()
        central_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
