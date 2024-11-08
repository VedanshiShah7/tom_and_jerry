import numpy as np

class PotentialFieldNavigator:
    def __init__(self, goal, repulsive_gain=100, attractive_gain=10, influence_radius=0.5):
        self.goal = goal
        self.repulsive_gain = repulsive_gain
        self.attractive_gain = attractive_gain
        self.influence_radius = influence_radius

    def calculate_force(self, position, obstacles):
        attractive_force = self.attractive_gain * np.array([self.goal[0] - position[0], self.goal[1] - position[1]])
        repulsive_force = np.array([0.0, 0.0])

        for obstacle in obstacles:
            distance = np.linalg.norm(np.array([obstacle[0] - position[0], obstacle[1] - position[1]]))
            if distance < self.influence_radius:
                repulsive_force += self.repulsive_gain * (1 / distance - 1 / self.influence_radius) * (np.array([position[0] - obstacle[0], position[1] - obstacle[1]]) / distance**2)

        total_force = attractive_force + repulsive_force
        return total_force
