from data.graph_weighted import ethiopia_graph
from uniform_cost_search.ucs import uniform_cost_search

start = "Addis Ababa"
goal = "Lalibela"

path, cost = uniform_cost_search(ethiopia_graph, start, goal)

print("UCS Path from Addis Ababa to Lalibela:")
print(path)
print("Total Cost:", cost)
