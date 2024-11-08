import heapq
from math import sqrt

class AStarPlanner:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.open_list = []
        self.came_from = {}
        self.g_cost = {start: 0}
        self.f_cost = {start: self.heuristic(start, goal)}
        heapq.heappush(self.open_list, (self.f_cost[start], start))

    def heuristic(self, current, goal):
        return sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

    def neighbors(self, node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        result = []
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if 0 <= neighbor[0] < len(self.grid) and 0 <= neighbor[1] < len(self.grid[0]) and self.grid[neighbor[0]][neighbor[1]] == 0:
                result.append(neighbor)
        return result

    def search(self):
        while self.open_list:
            current = heapq.heappop(self.open_list)[1]
            if current == self.goal:
                return self.reconstruct_path(current)

            for neighbor in self.neighbors(current):
                tentative_g_cost = self.g_cost[current] + 1
                if neighbor not in self.g_cost or tentative_g_cost < self.g_cost[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_cost[neighbor] = tentative_g_cost
                    self.f_cost[neighbor] = tentative_g_cost + self.heuristic(neighbor, self.goal)
                    heapq.heappush(self.open_list, (self.f_cost[neighbor], neighbor))

        return []

    def reconstruct_path(self, current):
        path = []
        while current in self.came_from:
            path.append(current)
            current = self.came_from[current]
        path.reverse()
        return path
