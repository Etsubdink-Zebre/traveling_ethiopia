import heapq

class AStarAgent:
    def __init__(self, graph, heuristic):
        self.graph = graph
        self.heuristic = heuristic

    def search(self, start, goal):
        """
        A* search algorithm from start to goal
        Returns path and total cost
        """
        frontier = []
        heapq.heappush(frontier, (self.heuristic[start], 0, start, [start]))
        explored = set()

        while frontier:
            f_cost, g_cost, current, path = heapq.heappop(frontier)

            if current == goal:
                return path, g_cost

            if current in explored:
                continue

            explored.add(current)

            for neighbor, edge_cost in self.graph[current].items():
                if neighbor not in explored:
                    g_new = g_cost + edge_cost
                    f_new = g_new + self.heuristic.get(neighbor, 0)
                    heapq.heappush(frontier, (f_new, g_new, neighbor, path + [neighbor]))

        return None, float('inf')
