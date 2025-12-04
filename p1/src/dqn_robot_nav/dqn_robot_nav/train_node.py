#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor


class DQNTrainingNode(Node):

    def __init__(self):
        super().__init__("dqn_training_node")

        self.n_episodes = 400
        self.max_steps_per_episode = 1200   # pequeño ajuste para más éxito
        self.state_size = 12
        self.action_size = 5

        self.env = TurtleBot3Env()
        self.state_processor = StateProcessor(n_lidar_bins=10)

        self.agent = DQNAgent(
            state_size=self.state_size,
            action_size=self.action_size,
            learning_rate=0.001,
            gamma=0.99,
            epsilon=1.0,
            epsilon_min=0.05,
            epsilon_decay=0.997,     # CAMBIO SUAVE
            memory_size=20000,
            batch_size=64
        )

        self.reward_history = []
        self.epsilon_history = []
        self.loss_history = []

        self.success_count = 0
        self.collision_count = 0

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


    def train(self):
        self.get_logger().info("Starting DQN Training...")

        for episode in range(self.n_episodes):

            # RESET
            self.env.reset(random_goal=True)

            state = None
            while state is None:
                state = self.get_processed_state()

            total_reward = 0

            for step in range(self.max_steps_per_episode):

                action = self.agent.act(state, training=True)
                loss = None

                _, reward, done = self.env.step(action)

                next_state = None
                while next_state is None:
                    next_state = self.get_processed_state()

                # Guardar experiencia
                self.agent.remember(state, action, reward, next_state, done)
                
                # Entrenamiento DQN
                loss = self.agent.replay()

                # Guardar métricas
                total_reward += reward
                self.loss_history.append(loss)

                state = next_state

                if done:
                    break

            # Registrar métricas por episodio
            self.reward_history.append(total_reward)
            self.epsilon_history.append(self.agent.epsilon)

            if self.env.is_goal_reached():
                self.success_count += 1
            if self.env.is_collision():
                self.collision_count += 1

            if episode % 10 == 0:
                success_rate = self.success_count / (episode + 1) * 100
                self.get_logger().info(
                    f"[EP {episode}] Reward={total_reward:.2f} | "
                    f"Eps={self.agent.epsilon:.2f} | "
                    f"Success={success_rate:.1f}%"
                )

        self.agent.save("trained_model3.pkl")
        self.get_logger().info("Model saved")

        # --- REWARD CURVE ---
        plt.figure()
        plt.plot(self.reward_history)
        plt.title("Reward per Episode")
        plt.xlabel("Episode")
        plt.ylabel("Total Reward")
        plt.grid(True)
        plt.savefig("reward_curve.jpg")
        plt.close()

        # --- EPSILON DECAY ---
        plt.figure()
        plt.plot(self.epsilon_history)
        plt.title("Epsilon Decay")
        plt.xlabel("Episode")
        plt.ylabel("Epsilon")
        plt.grid(True)
        plt.savefig("epsilon_decay.jpg")
        plt.close()

        # --- LOSS CURVE ---
        plt.figure()
        plt.plot(self.loss_history)
        plt.title("Training Loss")
        plt.xlabel("Training Step")
        plt.ylabel("Loss")
        plt.grid(True)
        plt.savefig("loss_curve.jpg")
        plt.close()

        self.get_logger().info("Gráficas guardadas: reward_curve.jpg, epsilon_decay.jpg, loss_curve.jpg")
        self.get_logger().info("TRAINING COMPLETED")


def main(args=None):
    rclpy.init(args=args)
    node = DQNTrainingNode()

    try:
        node.train()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
