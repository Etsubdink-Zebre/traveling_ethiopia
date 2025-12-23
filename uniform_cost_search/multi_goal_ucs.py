from uniform_cost_search.ucs import uniform_cost_search

def multi_goal_ucs(graph, start, goals):
    """
    Customized Uniform Cost Search for multiple goals.
    At each step, it visits the nearest unvisited goal using UCS.
    """
    current = start
    remaining_goals = set(goals)

    full_path = [start]
    total_cost = 0

    while remaining_goals:
        nearest_goal = None
        best_path = None
        best_cost = float('inf')

        # Find the closest goal from current location
        for goal in remaining_goals:
            path, cost = uniform_cost_search(graph, current, goal)
            if path and cost < best_cost:
                nearest_goal = goal
                best_path = path
                best_cost = cost

        if nearest_goal is None:
            break  # no reachable goals left

        # Append path (skip first node to avoid duplication)
        full_path.extend(best_path[1:])
        total_cost += best_cost
        current = nearest_goal
        remaining_goals.remove(nearest_goal)

    return full_path, total_cost
