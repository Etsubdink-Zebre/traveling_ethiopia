import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from data.graph_unweighted import ethiopia_graph
from search_agent import SearchAgent

agent = SearchAgent(ethiopia_graph)

start = "Addis Ababa"
goal = "Lalibela"

print("BFS Path:")
print(agent.search(start, goal, "bfs"))

print("\nDFS Path:")
print(agent.search(start, goal, "dfs"))
