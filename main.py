import random
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def parse_input():
    # Read start and goal co-ords
    start_goal_str = input().strip()
    start_str, goal_str = start_goal_str.split(";")
    start = tuple(map(int, start_str.split(",")))
    goal = tuple(map(int, goal_str.split(",")))

    # Read rectangular obstacles as the co-ords of their top-left and bottom-right corner
    obstacles = []
    while True:
        line = input().strip()
        if line == "-1":
            break
        x1, y1 = map(int, line.split(";")[0].split(","))
        x2, y2 = map(int, line.split(";")[1].split(","))
        # Normalise co-ords into this format:
        # (x_min, y_min, x_max, y_max)
        obstacles.append((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))
    
    return start, goal, obstacles

def do_lines_intersect(p1, p2, q1, q2):
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
    return (ccw(p1, q1, q2) != ccw(p2, q1, q2)) and (ccw(p1, p2, q1) != ccw(p1, p2, q2))

# Checks if the line from (x1, y1) to (x2, y2) intersects any of the given rectangular obstacles.
def is_collision(x1, y1, x2, y2, obstacles):
    for ox1, oy1, ox2, oy2 in obstacles:
        # Rectangle corners
        top_left = (ox1, oy1)
        top_right = (ox2, oy1)
        bottom_right = (ox2, oy2)
        bottom_left = (ox1, oy2)

        # Rectangle edges
        edges = [
            (top_left, top_right),
            (top_right, bottom_right),
            (bottom_right, bottom_left),
            (bottom_left, top_left),
        ]

        path_segment = ((x1, y1), (x2, y2))

        # Check if the path intersects any edge
        for edge in edges:
            if do_lines_intersect(path_segment[0], path_segment[1], edge[0], edge[1]):
                return True

        # Check if either endpoint is inside the obstacle
        if ox1 <= x1 <= ox2 and oy1 <= y1 <= oy2:
            return True
        if ox1 <= x2 <= ox2 and oy1 <= y2 <= oy2:
            return True

    return False

# Finds the node in the current tree closest to the sampled point using Euclidean distance
def get_nearest(node_list, x, y):
    return min(node_list, key=lambda node: (node.x - x) ** 2 + (node.y - y) ** 2)

def rrt(start, goal, obstacles, max_iter=10000, step_size=5):
    # Initialise tree with root at start
    nodes = []
    root = Node(*start)
    nodes.append(root)
    goal_node = None

    # Randomly sample a point.
    for _ in range(max_iter):
        if random.random() < 0.1:
            rx, ry = goal # 10% of the time, bias towards the goal (exploration bias)
        else:
            rx = random.randint(0, 100)
            ry = random.randint(0, 100)
        
        # Extend a small step from the nearest node toward the random point.
        nearest = get_nearest(nodes, rx, ry)
        theta = math.atan2(ry - nearest.y, rx - nearest.x)
        new_x = nearest.x + step_size * math.cos(theta)
        new_y = nearest.y + step_size * math.sin(theta)

        # If the path hits an obstacle, skip this extension.
        if is_collision(nearest.x, nearest.y, new_x, new_y, obstacles):
            continue
        
        # Otherwise, add the new node to the tree
        new_node = Node(new_x, new_y, nearest)
        nodes.append(new_node)

        # If the new node is close to the goal and the path to the goal is clear, finish by adding the goal node.
        if math.hypot(new_node.x - goal[0], new_node.y - goal[1]) < step_size:
            if not is_collision(new_node.x, new_node.y, goal[0], goal[1], obstacles):
                goal_node = Node(goal[0], goal[1], new_node)
                break
    
    # If goal wasn't reached, return empty list.
    if not goal_node:
        print("No path found.")
        return [], nodes
    
    # Trace back from goal to root using parent pointers to construct the path.
    path = []
    current = goal_node
    while current:
        path.append((round(current.x), round(current.y)))
        current = current.parent
    path.reverse()
    return path, nodes

def plot_path(start, goal, obstacles, path, nodes):
    fig, ax = plt.subplots()

    # Plot all tree edges
    for node in nodes:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], color='gray', linewidth=0.5)

    # Plot start and goal
    ax.plot(start[0], start[1], 'go', markersize=8, label='Start')
    ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')

    # Plot obstacles
    for (x1, y1, x2, y2) in obstacles:
        ax.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, color='black'))

    # Plot final path
    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color='blue', linewidth=2, label='Path')

    ax.set_aspect('equal')
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.legend()
    plt.title("RRT Path Planning")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    start, goal, obstacles = parse_input()
    path, nodes = rrt(start, goal, obstacles)
    for x, y in path:
        print(f"{x},{y}")
    plot_path(start, goal, obstacles, path, nodes)
