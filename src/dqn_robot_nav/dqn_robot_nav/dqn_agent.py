import numpy as np
from sklearn.neural_network import MLPRegressor
from collections import deque
import random
import pickle

class DQNAgent:
    """Deep Q-Network agent using sklearn's MLPRegressor"""

    def __init__(self,
                 state_size,
                 action_size,
                 learning_rate=0.001,
                 gamma=0.99,
                 epsilon=1.0,
                 epsilon_min=0.05,
                 epsilon_decay=0.997,     # ← CAMBIO SUAVE PARA MÁS EXPLORACIÓN
                 memory_size=20000,
                 batch_size=64,
                 target_update_freq=200):

        self.state_size = state_size
        self.action_size = action_size
        self.gamma = gamma

        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay

        self.batch_size = batch_size
        self.target_update_freq = target_update_freq
        self.step_count = 0

        self.memory = deque(maxlen=memory_size)

        # Main Q-network
        self.q_network = MLPRegressor(
            hidden_layer_sizes=(128, 128),
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,
            warm_start=True,
            random_state=42
        )

        # Target Q-network
        self.target_network = MLPRegressor(
            hidden_layer_sizes=(128, 128),
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,
            warm_start=True,
            random_state=42
        )

        # Initialize with dummy training
        X = np.random.randn(10, state_size)
        y = np.random.randn(10, action_size)

        self.q_network.fit(X, y)
        self.target_network.fit(X, y)

    # -------------------------------------------------------

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    # -------------------------------------------------------

    def act(self, state, training=True):
        if training and np.random.rand() < self.epsilon:
            return random.randrange(self.action_size)

        q_vals = self.q_network.predict(state.reshape(1, -1))[0]
        return np.argmax(q_vals)

    # -------------------------------------------------------

    def replay(self):
        if len(self.memory) < self.batch_size:
            return 0.0

        minibatch = random.sample(self.memory, self.batch_size)

        states = np.vstack([m[0] for m in minibatch])
        actions = np.array([m[1] for m in minibatch])
        rewards = np.array([m[2] for m in minibatch])
        next_states = np.vstack([m[3] for m in minibatch])
        dones = np.array([m[4] for m in minibatch])

        q_current = self.q_network.predict(states)
        q_next = self.target_network.predict(next_states)

        for i in range(self.batch_size):
            if dones[i]:
                q_current[i][actions[i]] = rewards[i]
            else:
                q_current[i][actions[i]] = rewards[i] + self.gamma * np.max(q_next[i])

        self.q_network.partial_fit(states, q_current)

        self.step_count += 1
        if self.step_count % self.target_update_freq == 0:
            self.update_target_network()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        return float(np.mean((q_current - self.q_network.predict(states))**2))

    # -------------------------------------------------------

    def update_target_network(self):
        self.target_network = pickle.loads(pickle.dumps(self.q_network))
        print("[DQN] Target network updated")

    # -------------------------------------------------------

    def save(self, filepath):
        data = {
            "q_network": self.q_network,
            "target_network": self.target_network,
            "epsilon": self.epsilon,
            "step_count": self.step_count
        }
        with open(filepath, "wb") as f:
            pickle.dump(data, f)
        print(f"[DQN] Model saved to {filepath}")

    def load(self, filepath):
        with open(filepath, "rb") as f:
            data = pickle.load(f)

        self.q_network = data["q_network"]
        self.target_network = data["target_network"]
        self.epsilon = data["epsilon"]
        self.step_count = data["step_count"]

        print(f"[DQN] Model loaded from {filepath}")
