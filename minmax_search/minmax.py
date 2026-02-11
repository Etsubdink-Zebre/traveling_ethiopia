from data.adversarial_graph import graph, utility


class MiniMaxSearch:
    def __init__(self, graph, utility):
        self.graph = graph
        self.utility = utility

    def is_terminal(self, state):
        return not self.graph.get(state, [])

    def max_value(self, state, depth=0, max_depth=10):
        if self.is_terminal(state) or depth >= max_depth:
            return self.utility.get(state, 0), state

        best_val, best_move = float('-inf'), None
        for neighbor in self.graph.get(state, []):
            val, _ = self.min_value(neighbor, depth + 1, max_depth)
            if val > best_val:
                best_val, best_move = val, neighbor
        return best_val, best_move

    def min_value(self, state, depth, max_depth):
        if self.is_terminal(state) or depth >= max_depth:
            return self.utility.get(state, 0), state

        best_val, best_move = float('inf'), None
        for neighbor in self.graph.get(state, []):
            val, _ = self.max_value(neighbor, depth + 1, max_depth)
            if val < best_val:
                best_val, best_move = val, neighbor
        return best_val, best_move

    def find_best_path(self, start_state, max_depth=10):
        current = start_state
        path = [current]

        while not self.is_terminal(current) and len(path) <= max_depth:
            if len(path) % 2 == 1:
                _, next_state = self.max_value(current, len(path), max_depth)
            else:
                _, next_state = self.min_value(current, len(path), max_depth)

            if next_state is None or next_state == current:
                break

            current = next_state
            path.append(current)

        return self.utility.get(current, 0), path


if __name__ == "__main__":
    minimax = MiniMaxSearch(graph, utility)
    start = "Addis Ababa"
    best_utility, path = minimax.find_best_path(start)

    print(f"Starting from: {start}")
    print(f"Best path: {' -> '.join(path)}")
    print(f"Final utility (coffee quality): {best_utility}")
