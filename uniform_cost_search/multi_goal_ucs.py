from uniform_cost_search.ucs import uniform_cost_search


def multi_goal_ucs(graph, start, goals):
    """Visit every goal city using greedy nearest-unvisited via UCS."""
    current = start
    remaining = set(goals)
    full_path = [start]
    total_cost = 0

    while remaining:
        nearest, best_path, best_cost = None, None, float('inf')

        for goal in remaining:
            path, cost = uniform_cost_search(graph, current, goal)
            if path and cost < best_cost:
                nearest, best_path, best_cost = goal, path, cost

        if nearest is None:
            break

        full_path.extend(best_path[1:])
        total_cost += best_cost
        current = nearest
        remaining.remove(nearest)

    return full_path, total_cost
