from data.graph_unweighted import ethiopia_graph
from uninformed_search.search_agent import SearchAgent

agent = SearchAgent(ethiopia_graph)

start = "Addis Ababa"
goal = "Lalibela"

print("BFS Path:")
print(agent.search(start, goal, "bfs"))

print("\nDFS Path:")
print(agent.search(start, goal, "dfs"))
