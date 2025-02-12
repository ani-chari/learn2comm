#!/usr/bin/env python3
"""
This simulation demonstrates a multi-robot warehouse scenario that leverages RMF’s
built-in warehouse environment (e.g. rmf_demos_warehouse) as well as its traffic management
and crowd simulation capabilities.

Robots (each a ROS node) do the following:
  • Move in the warehouse according to coordinated motion (here simulated as a bounded
    random walk to mimic RMF traffic management).
  • Continuously “sense” dynamic obstacles (e.g. humans) with realistic noise.
    (Obstructions are assumed to be handled by the RMF environment; here we simply ignore
    measurements beyond a sensing range.)
  • Maintain local beliefs (a simple record of the latest “good” observation) about each dynamic
    obstacle.
  • Periodically publish (upload) their current state, sensor observations, and beliefs.
    
The central observation manager node collects these uploads at a fixed frequency and,
using a (placeholder) selection algorithm that you have already designed, chooses which
observations should be shared from “good” (accurate) robots to those with noisier views.
This process minimizes overall bandwidth consumption while still ensuring that robots
that need reliable obstacle data receive it.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import random
import math

# ---- Simulation Parameters ----
WAREHOUSE_SIZE = 20.0         # Warehouse boundaries: 0 to 20 m in x and y.
SENSING_RANGE = 10.0          # Maximum range (m) at which a dynamic obstacle may be detected.
NOISE_STDDEV = 0.5            # Standard deviation (m) of Gaussian noise added to sensed obstacle positions.
OBSERVATION_INTERVAL = 1.0    # Seconds between each robot’s sensor update and state upload.
COMMUNICATION_INTERVAL = 5.0  # Seconds between each centralized observation exchange.

# ---- A simple sensor model for dynamic obstacles ----
def simulate_sensor_measurement(robot_pos, obstacle_pos):
    """Simulate a sensor reading for a single dynamic obstacle.
       Returns a tuple (noisy_x, noisy_y, true_distance) if within range, else None."""
    dx = robot_pos[0] - obstacle_pos[0]
    dy = robot_pos[1] - obstacle_pos[1]
    distance = math.hypot(dx, dy)
    if distance > SENSING_RANGE:
        return None  # Out-of-range: no observation produced.
    # In a realistic sensor the measurement is noisy.
    noisy_x = obstacle_pos[0] + random.gauss(0.0, NOISE_STDDEV)
    noisy_y = obstacle_pos[1] + random.gauss(0.0, NOISE_STDDEV)
    return (noisy_x, noisy_y, distance)

# ---- Global simulation for dynamic obstacles (crowd simulation) ----
# In an RMF simulation, these would be simulated by RMF’s crowd simulation plugins.
DYNAMIC_OBSTACLES = [
    {'id': 'human_1', 'true_position': [5.0, 5.0]},
    {'id': 'human_2', 'true_position': [15.0, 10.0]}
]

def update_dynamic_obstacles():
    """Simple update: each dynamic obstacle makes a small move; boundaries are enforced."""
    for obs in DYNAMIC_OBSTACLES:
        x, y = obs['true_position']
        dx = random.uniform(-0.2, 0.2)
        dy = random.uniform(-0.2, 0.2)
        new_x = min(max(x + dx, 0.0), WAREHOUSE_SIZE)
        new_y = min(max(y + dy, 0.0), WAREHOUSE_SIZE)
        obs['true_position'] = [new_x, new_y]

def get_dynamic_obstacles():
    """Returns the current list of dynamic obstacles."""
    return DYNAMIC_OBSTACLES

# ---- Robot Node Definition ----
class RobotNode(Node):
    def __init__(self, robot_id: int):
        name = f'robot_{robot_id}'
        super().__init__(name)
        self.robot_id = robot_id
        # Initialize the robot’s state with a random starting position
        self.state = {'position': [random.uniform(0, WAREHOUSE_SIZE), random.uniform(0, WAREHOUSE_SIZE)]}
        # Beliefs will be maintained as a dict mapping obstacle_id to the latest (x,y) position.
        self.beliefs = {}
        # Publisher to broadcast robot state, observations, and beliefs
        self.obs_pub = self.create_publisher(String, 'robot_observations', 10)
        # Subscriber to receive shared observations from the central server
        self.shared_sub = self.create_subscription(String, 'shared_observations', self.shared_callback, 10)
        # Timer for periodic sensor updates and state uploads
        self.timer = self.create_timer(OBSERVATION_INTERVAL, self.timer_callback)

    def timer_callback(self):
        # Update the robot’s state using a simple (RMF-inspired) coordinated movement.
        self.simulate_movement()
        # For each dynamic obstacle, run the sensor simulation.
        observations = []
        for obs in get_dynamic_obstacles():
            measurement = simulate_sensor_measurement(self.state['position'], obs['true_position'])
            if measurement:
                observation = {
                    'robot_id': self.robot_id,
                    'obstacle_id': obs['id'],
                    'measured_position': {'x': measurement[0], 'y': measurement[1]},
                    'distance': measurement[2],
                    'timestamp': self.get_clock().now().nanoseconds  # could also use .sec
                }
                # For simplicity, update local belief by simply using the newest measurement.
                self.beliefs[obs['id']] = observation['measured_position']
                observations.append(observation)
        # Bundle the robot’s state, observations, and current beliefs into one message.
        msg_dict = {
            'robot_id': self.robot_id,
            'state': {'position': {'x': self.state['position'][0], 'y': self.state['position'][1]}},
            'observations': observations,
            'beliefs': self.beliefs
        }
        json_msg = String()
        json_msg.data = json.dumps(msg_dict)
        self.obs_pub.publish(json_msg)
        self.get_logger().info(f'Robot {self.robot_id} published state: {json_msg.data}')

    def simulate_movement(self):
        """
        Simulate coordinated motion using RMF’s traffic management features.
        In a full implementation you would make service calls or use RMF APIs to compute safe
        trajectories; here we use a simple bounded random walk.
        """
        pos = self.state['position']
        # Compute a small random displacement
        delta = [random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5)]
        new_x = min(max(pos[0] + delta[0], 0.0), WAREHOUSE_SIZE)
        new_y = min(max(pos[1] + delta[1], 0.0), WAREHOUSE_SIZE)
        self.state['position'] = [new_x, new_y]

    def shared_callback(self, msg: String):
        """
        Callback for processing shared observations received from the central communication node.
        Upon receipt, update the robot’s local belief for the referenced obstacle.
        """
        try:
            data = json.loads(msg.data)
            sender = data.get('from_robot')
            obs = data.get('observation')
            if sender is not None and obs is not None:
                obstacle_id = obs.get('obstacle_id')
                received_pose = obs.get('measured_position')
                # Here one might fuse the received observation with current belief; for now we simply override.
                self.beliefs[obstacle_id] = received_pose
                self.get_logger().info(
                    f'Robot {self.robot_id} updated belief for obstacle {obstacle_id} from robot {sender}: {received_pose}'
                )
        except Exception as e:
            self.get_logger().error(f'Error processing shared observation: {e}')

# ---- Central Observation Manager Node Definition ----
class CentralObservationManager(Node):
    def __init__(self):
        super().__init__('central_observation_manager')
        # Subscriber to collect all robot uploads
        self.data_sub = self.create_subscription(String, 'robot_observations', self.data_callback, 10)
        # Publisher to send selected observations to individual robots
        self.share_pub = self.create_publisher(String, 'shared_observations', 10)
        # Dictionary to cache the latest upload for each robot (keyed by robot_id)
        self.robot_data = {}
        self.comm_timer = self.create_timer(COMMUNICATION_INTERVAL, self.comm_callback)

    def data_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            robot_id = data.get('robot_id')
            if robot_id is not None:
                self.robot_data[str(robot_id)] = data
                self.get_logger().info(f'Central collected data from robot {robot_id}.')
        except Exception as e:
            self.get_logger().error(f'Error parsing robot observation: {e}')

    def comm_callback(self):
        """
        At a fixed frequency, run the observation-sharing algorithm to decide between pairs of robots.
        For each obstacle, if one robot’s observation is noisy (e.g. measured distance > threshold)
        and another robot has a “better” (closer or more certain) observation, send that observation to
        update the weaker robot’s belief.
        
        (Replace the following simple procedure with your actual combinatorial or RL algorithm.)
        """
        self.get_logger().info("Central running observation selection algorithm...")
        # Loop over each robot’s data.
        for rec_robot_id, data in self.robot_data.items():
            # For each observation of this robot, check if it is “poor” (distance > 5 m).
            for obs in data.get('observations', []):
                obs_id = obs.get('obstacle_id')
                rec_distance = obs.get('distance', float('inf'))
                if rec_distance > 5.0:
                    # Search other robots for a better observation of the same dynamic obstacle.
                    for send_robot_id, other_data in self.robot_data.items():
                        if send_robot_id == rec_robot_id:
                            continue
                        for other_obs in other_data.get('observations', []):
                            if other_obs.get('obstacle_id') == obs_id and other_obs.get('distance', float('inf')) < rec_distance:
                                # Found a better observation; send it to the receiving robot.
                                share_msg = {
                                    'from_robot': send_robot_id,
                                    'to_robot': rec_robot_id,
                                    'observation': other_obs
                                }
                                json_msg = String()
                                json_msg.data = json.dumps(share_msg)
                                self.share_pub.publish(json_msg)
                                self.get_logger().info(
                                    f'Shared observation for obstacle {obs_id} from robot {send_robot_id} to robot {rec_robot_id}'
                                )
                                break
        # (Optionally, you could also clear self.robot_data here to only process fresh data each cycle.)
        self.robot_data.clear()

# ---- Main Function ----
def main(args=None):
    rclpy.init(args=args)
    # Create a few robot nodes. In a full RMF simulation these would be coordinated via rmf_traffic.
    num_robots = 3
    robot_nodes = [RobotNode(robot_id=i + 1) for i in range(num_robots)]
    # The central observation manager
    central_node = CentralObservationManager()

    # Use a MultiThreadedExecutor so our nodes can process in parallel.
    executor = MultiThreadedExecutor()
    for node in robot_nodes:
        executor.add_node(node)
    executor.add_node(central_node)

    try:
        # Run the simulation loop.
        while rclpy.ok():
            # Update the dynamic obstacles (simulating RMF crowd simulation)
            update_dynamic_obstacles()
            # Spin each node for a short time.
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Shutting down simulation.")
    finally:
        for node in robot_nodes:
            node.destroy_node()
        central_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
