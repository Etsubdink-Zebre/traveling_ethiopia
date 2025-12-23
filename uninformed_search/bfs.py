from collections import deque

def bfs(graph, start, goal):
    """
    Breadth-First Search (FIFO queue)
    Returns a path from start to goal if found.
    """
    queue = deque([[start]])
    visited = set()

    while queue:
        path = queue.popleft()
        current = path[-1]

        if current == goal:
            return path

        if current not in visited:
            visited.add(current)

            for neighbor in graph.get(current, []):
                if neighbor not in visited:
                    queue.append(path + [neighbor])

    return None
