import heapq

def uniform_cost_search(graph, start, goal):
    """
    Uniform Cost Search (UCS)
    """
    frontier = []
    heapq.heappush(frontier, (0, start, [start]))

    explored = set()

    while frontier:
        cost, current, path = heapq.heappop(frontier)

        if current == goal:
            return path, cost

        if current in explored:
            continue

        explored.add(current)

        for neighbor, edge_cost in graph[current].items():
            if neighbor not in explored:
                heapq.heappush(
                    frontier,
                    (cost + edge_cost, neighbor, path + [neighbor])
                )

    return None, float('inf')
