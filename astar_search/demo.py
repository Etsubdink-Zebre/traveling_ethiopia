from data.graph_weighted import ethiopia_graph
from astar_search.astar import AStarAgent
from data.heuristic import ethiopia_heuristic_graph  # updated heuristic

start = "Addis Ababa"
goal = "Moyale"

agent = AStarAgent(ethiopia_graph, ethiopia_heuristic_graph)
path, cost = agent.search(start, goal)

print("A* Path from Addis Ababa to Moyale:")
print(path)
print("Total Cost:", cost)
