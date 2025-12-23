from data.graph_weighted import ethiopia_graph
from uniform_cost_search.multi_goal_ucs import multi_goal_ucs

start = "Addis Ababa"
goals = [
    "Axum",
    "Gondar",
    "Lalibela",
    "Babile",
    "Jimma",
    "Bale",
    "Sof Oumer",
    "Arba Minch"
]

path, cost = multi_goal_ucs(ethiopia_graph, start, goals)

print("Multi-Goal UCS Path:")
print(path)
print("Total Cost:", cost)
