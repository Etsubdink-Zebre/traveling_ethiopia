from uninformed_search.bfs import bfs
from uninformed_search.dfs import dfs


class SearchAgent:
    """
    Search agent that can perform BFS or DFS
    on the Traveling Ethiopia state space graph.
    """

    def __init__(self, graph):
        self.graph = graph

    def search(self, initial_state, goal_state, strategy):
        strategy = strategy.lower()

        if strategy == "bfs":
            return bfs(self.graph, initial_state, goal_state)

        elif strategy == "dfs":
            return dfs(self.graph, initial_state, goal_state)

        else:
            raise ValueError("Only 'bfs' and 'dfs' strategies are supported.")
