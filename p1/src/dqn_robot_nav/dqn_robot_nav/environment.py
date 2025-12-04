import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_srvs.srv import Empty
import numpy as np
import math
import os


class TurtleBot3Env(Node):

    def __init__(self):
        super().__init__("turtlebot3_env")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.reset_world_client = self.create_client(Empty, "/reset_world")
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")

        self.scan_data = None
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.last_distance = None

        self.goal_position = (0.0, 0.0)
        self.goal_model_name = "goal_marker_red"

        self.model_sdf_path = os.path.expanduser(
            "/home/dylan/repos/second_test_robotic/src/dqn_robot_nav/models/goal_red/model.sdf"
        )

        if not os.path.exists(self.model_sdf_path):
            self.get_logger().error(f"SDF model not found: {self.model_sdf_path}")
        else:
            self.get_logger().info(f"Loaded model: {self.model_sdf_path}")

        self.actions = {
            0: (0.10, 0.0),
            1: (0.0, 0.6),
            2: (0.0, -0.6),
            3: (0.07, 0.3),
            4: (0.07, -0.3),
        }

        self.collision_threshold = 0.20
        self.goal_threshold = 0.25

    def scan_callback(self, msg):
        arr = np.array(msg.ranges)
        arr = np.where(np.isinf(arr), msg.range_max, arr)
        self.scan_data = np.nan_to_num(arr)

    def odom_callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation
        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def sample_goal_inside_arena(self):
        while True:
            gx = np.random.uniform(-1.8, 1.8)
            gy = np.random.uniform(-1.6, 1.6)

            # avoid central dense obstacle region
            if (-0.6 < gx < 0.6 and -0.8 < gy < 0.8):
                continue

            return (gx, gy)

    def delete_old_goal(self):
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            return

        req = DeleteEntity.Request()
        req.name = self.goal_model_name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def spawn_new_goal(self):

        if not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("spawn_entity service unavailable")
            return

        with open(self.model_sdf_path, "r") as f:
            sdf_xml = f.read()

        gx, gy = self.goal_position

        req = SpawnEntity.Request()
        req.name = self.goal_model_name
        req.xml = sdf_xml
        req.initial_pose.position.x = gx
        req.initial_pose.position.y = gy
        req.initial_pose.position.z = 0.05

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f"Spawned GOAL at ({gx:.2f}, {gy:.2f})")

    def reset_world(self):
        if not self.reset_world_client.wait_for_service(timeout_sec=5.0):
            return
        req = Empty.Request()
        future = self.reset_world_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def reset(self, random_goal=True):

        self.send_velocity(0.0, 0.0)
        self.reset_world()

        self.delete_old_goal()

        if random_goal:
            self.goal_position = self.sample_goal_inside_arena()

        self.spawn_new_goal()
        self.last_distance = self.distance_to_goal()

    def step(self, action):
        lin, ang = self.actions[action]
        self.send_velocity(lin, ang)

        rclpy.spin_once(self, timeout_sec=0.08)

        reward, done = self.compute_reward()
        return None, reward, done

    def compute_reward(self):

        if self.is_collision():
            return -100.0, True

        if self.is_goal_reached():
            return 200.0, True

        dist = self.distance_to_goal()

        if self.last_distance is None:
            self.last_distance = dist

        progress = (self.last_distance - dist) * 15.0
        self.last_distance = dist

        # Orientation reward
        gx, gy = self.goal_position
        rx, ry = self.position
        dx = gx - rx
        dy = gy - ry

        goal_theta = math.atan2(dy, dx)
        angle_error = ((goal_theta - self.yaw + np.pi) % (2*np.pi)) - np.pi

        orientation_reward = -abs(angle_error) * 0.1
        heading_alignment = math.cos(angle_error)
        heading_bonus = heading_alignment * 1.5

        min_d = np.min(self.scan_data)
        obstacle_penalty = -(0.4 - min_d) * 0.5 if min_d < 0.4 else 0.0
        time_penalty = -0.005

        reward = (
            progress +
            orientation_reward +
            heading_bonus +
            obstacle_penalty +
            time_penalty
        )

        return reward, False

    def is_collision(self):
        return np.min(self.scan_data) < self.collision_threshold

    def is_goal_reached(self):
        return self.distance_to_goal() < self.goal_threshold

    def distance_to_goal(self):
        gx, gy = self.goal_position
        rx, ry = self.position
        return math.dist((gx, gy), (rx, ry))

    def send_velocity(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_vel_pub.publish(msg)
