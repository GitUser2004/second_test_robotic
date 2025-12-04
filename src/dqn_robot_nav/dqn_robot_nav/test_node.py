#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor


class DQNTestNode(Node):

    def __init__(self):
        super().__init__("dqn_test_node")

        self.state_size = 12
        self.action_size = 5

        self.env = TurtleBot3Env()
        self.state_processor = StateProcessor(n_lidar_bins=10)

        self.agent = DQNAgent(
            state_size=self.state_size,
            action_size=self.action_size,
            epsilon=0.0,       # greedy
            epsilon_min=0.0,
            epsilon_decay=1.0
        )

        try:
            self.agent.load("trained_model3.pkl")
            self.get_logger().info("Modelo cargado correctamente")
        except:
            self.get_logger().error("No se pudo cargar el modelo entrenado")
            return

        self.get_logger().info("Iniciando pruebas del modelo...")
        self.run_tests(n_tests=10)

    def get_processed_state(self):
        if self.env.scan_data is None:
            rclpy.spin_once(self.env, timeout_sec=0.1)
            return None

        try:
            return self.state_processor.get_state(
                self.env.scan_data,
                self.env.position,
                self.env.goal_position,
                self.env.yaw
            )
        except:
            return None

    def run_single_episode(self, episode_num):
        max_steps = 1500

        self.env.reset(random_goal=True)
        rclpy.spin_once(self.env)

        state = None
        while state is None:
            state = self.get_processed_state()

        total_reward = 0

        for step in range(max_steps):

            action = self.agent.act(state, training=False)
            _, reward, done = self.env.step(action)
            total_reward += reward

            next_state = None
            while next_state is None:
                next_state = self.get_processed_state()

            state = next_state

            if done:
                break

        self.env.send_velocity(0.0, 0.0)

        if self.env.is_goal_reached():
            result = "SUCCESS"
            self.get_logger().info(f"Episode {episode_num}: GOAL | Reward={total_reward:.2f}")
        elif self.env.is_collision():
            result = "COLLISION"
            self.get_logger().info(f"Episode {episode_num}: COLLISION | Reward={total_reward:.2f}")
        else:
            result = "TIMEOUT"
            self.get_logger().info(f"Episode {episode_num}: TIMEOUT | Reward={total_reward:.2f}")

        return result, total_reward


    # ===================================================
    def run_tests(self, n_tests=10):
        """End of N tests."""

        results = []
        rewards = []

        for i in range(1, n_tests + 1):
            result, reward = self.run_single_episode(i)
            results.append(result)
            rewards.append(reward)

        successes = results.count("SUCCESS")
        collisions = results.count("COLLISION")
        timeouts = results.count("TIMEOUT")

        avg_reward = np.mean(rewards)
        std_reward = np.std(rewards)

        self.get_logger().info("\n" + "=" * 55)
        self.get_logger().info("Final results")
        self.get_logger().info(f"Evaluated: {n_tests}")
        self.get_logger().info(f"Wins      : {successes}")
        self.get_logger().info(f"Collisions  : {collisions}")
        self.get_logger().info(f"Timeouts    : {timeouts}")
        self.get_logger().info(f"Success Rate: {successes / n_tests * 100:.1f}%")
        self.get_logger().info(f"Avg Reward  : {avg_reward:.2f}")
        self.get_logger().info(f"Std Reward  : {std_reward:.2f}")
        self.get_logger().info("=" * 55 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = DQNTestNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
