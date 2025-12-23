def dfs(graph, start, goal):
    """
    Depth-First Search (LIFO stack)
    Returns a path from start to goal if found.
    """
    stack = [[start]]
    visited = set()

    while stack:
        path = stack.pop()
        current = path[-1]

        if current == goal:
            return path

        if current not in visited:
            visited.add(current)

            for neighbor in graph.get(current, []):
                if neighbor not in visited:
                    stack.append(path + [neighbor])

    return None
