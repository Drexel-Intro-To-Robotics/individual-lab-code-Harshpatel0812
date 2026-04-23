import sys
import math
import random
import time
from PIL import Image

# ------------------------------------------------------------
# Runtime variables
# ------------------------------------------------------------
start = (0, 0)
end = (0, 0)
difficulty = ""

# ------------------------------------------------------------
# Display colors
# ------------------------------------------------------------
TREE_COLOR = (0, 180, 0)        # Green tree edges
PATH_COLOR = (128, 0, 128)      # Purple final path
START_COLOR = (255, 0, 0)       # Red start
GOAL_COLOR = (0, 0, 255)        # Blue goal
SAMPLE_COLOR = (180, 180, 180)  # Optional sampled nodes (unused here)

# ------------------------------------------------------------
# RRT parameters
# ------------------------------------------------------------
STEP_SIZE = 10          # how far tree extends toward each sample
GOAL_RADIUS = 12        # if new node is this close to goal, connect to goal
GOAL_BIAS = 0.08        # probability of sampling the goal directly
MAX_ITER = 20000        # max iterations per run
RUNS = 100              # number of runs for statistics


# ------------------------------------------------------------
# Utility functions
# ------------------------------------------------------------
def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def in_bounds(point, width, height):
    x, y = point
    return 0 <= x < width and 0 <= y < height


def is_free(map_pixels, point):
    x, y = point
    return bool(map_pixels[x, y])


def nearest_node(tree_nodes, sample):
    best_node = tree_nodes[0]
    best_dist = euclidean(best_node, sample)

    for node in tree_nodes[1:]:
        d = euclidean(node, sample)
        if d < best_dist:
            best_dist = d
            best_node = node

    return best_node


def steer(from_node, to_node, step_size):
    dx = to_node[0] - from_node[0]
    dy = to_node[1] - from_node[1]
    dist = math.hypot(dx, dy)

    if dist == 0:
        return from_node

    if dist <= step_size:
        return (int(round(to_node[0])), int(round(to_node[1])))

    scale = step_size / dist
    new_x = from_node[0] + dx * scale
    new_y = from_node[1] + dy * scale

    return (int(round(new_x)), int(round(new_y)))


def collision_free(map_pixels, p1, p2, width, height):
    """
    Check whether the straight line from p1 to p2 stays inside free space.
    Uses dense interpolation along the segment.
    """
    dist = euclidean(p1, p2)
    steps = max(int(dist), 1)

    for i in range(steps + 1):
        t = i / steps
        x = int(round(p1[0] + t * (p2[0] - p1[0])))
        y = int(round(p1[1] + t * (p2[1] - p1[1])))

        if not in_bounds((x, y), width, height):
            return False
        if not is_free(map_pixels, (x, y)):
            return False

    return True


def reconstruct_path(parents, goal_node):
    path = []
    current = goal_node

    while current is not None:
        path.append(current)
        current = parents[current]

    path.reverse()
    return path


def path_cost(path):
    if len(path) < 2:
        return 0.0

    cost = 0.0
    for i in range(len(path) - 1):
        cost += euclidean(path[i], path[i + 1])
    return cost


def draw_line(pixel_access, p1, p2, color):
    """
    Draw line by dense interpolation.
    """
    dist = euclidean(p1, p2)
    steps = max(int(dist), 1)

    for i in range(steps + 1):
        t = i / steps
        x = int(round(p1[0] + t * (p2[0] - p1[0])))
        y = int(round(p1[1] + t * (p2[1] - p1[1])))
        pixel_access[x, y] = color


def draw_marker(pixel_access, point, color, width, height, size=2):
    px, py = point
    for dx in range(-size, size + 1):
        for dy in range(-size, size + 1):
            nx, ny = px + dx, py + dy
            if 0 <= nx < width and 0 <= ny < height:
                pixel_access[nx, ny] = color


# ------------------------------------------------------------
# RRT core algorithm
# ------------------------------------------------------------
def rrt_search(map_pixels, width, height, start, goal,
               step_size=STEP_SIZE,
               goal_radius=GOAL_RADIUS,
               goal_bias=GOAL_BIAS,
               max_iter=MAX_ITER):
    """
    Returns:
        success (bool)
        tree_nodes (list of nodes)
        parents (dict child -> parent)
        final_path (list of points)
        iterations_used (int)
    """
    tree_nodes = [start]
    parents = {start: None}

    if not is_free(map_pixels, start) or not is_free(map_pixels, goal):
        return False, tree_nodes, parents, [], 0

    for iteration in range(1, max_iter + 1):
        # Goal bias: sometimes sample the goal directly
        if random.random() < goal_bias:
            sample = goal
        else:
            sample = (random.randint(0, width - 1), random.randint(0, height - 1))

        # Skip sampled obstacle points
        if not is_free(map_pixels, sample):
            continue

        nearest = nearest_node(tree_nodes, sample)
        new_node = steer(nearest, sample, step_size)

        if new_node == nearest:
            continue

        if not in_bounds(new_node, width, height):
            continue

        if not is_free(map_pixels, new_node):
            continue

        if not collision_free(map_pixels, nearest, new_node, width, height):
            continue

        tree_nodes.append(new_node)
        parents[new_node] = nearest

        # Try to connect to goal if close enough
        if euclidean(new_node, goal) <= goal_radius:
            if collision_free(map_pixels, new_node, goal, width, height):
                tree_nodes.append(goal)
                parents[goal] = new_node
                final_path = reconstruct_path(parents, goal)
                return True, tree_nodes, parents, final_path, iteration

    return False, tree_nodes, parents, [], max_iter


# ------------------------------------------------------------
# Visualization
# ------------------------------------------------------------
def visualize_rrt(save_file, difficulty, tree_nodes, parents, final_path, start, goal):
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()
    width, height = im.size

    # Draw RRT tree
    for node in tree_nodes:
        parent = parents.get(node)
        if parent is not None:
            draw_line(pixel_access, parent, node, TREE_COLOR)

    # Draw final path
    for i in range(len(final_path) - 1):
        draw_line(pixel_access, final_path[i], final_path[i + 1], PATH_COLOR)

    draw_marker(pixel_access, start, START_COLOR, width, height, size=2)
    draw_marker(pixel_access, goal, GOAL_COLOR, width, height, size=2)

    im.save(save_file)

# ------------------------------------------------------------
# Statistics helpers
# ------------------------------------------------------------
def mean(values):
    if not values:
        return 0.0
    return sum(values) / len(values)


def std_dev(values):
    if len(values) < 2:
        return 0.0
    m = mean(values)
    variance = sum((v - m) ** 2 for v in values) / (len(values) - 1)
    return math.sqrt(variance)


# ------------------------------------------------------------
# Map setup
# ------------------------------------------------------------
def set_start_goal(difficulty):
    if difficulty == "maps/trivial.gif":
        return (8, 1), (20, 1)
    elif difficulty == "maps/medium.gif":
        return (8, 201), (110, 1)
    elif difficulty == "maps/very_hard.gif":
        return (1, 324), (580, 1)
    elif difficulty == "maps/my_maze.gif":
        return (0, 0), (500, 205)
    elif difficulty == "maps/my_maze2.gif":
        return (0, 0), (599, 350)
    elif difficulty == "maps/super_hard.gif":
        return (1, 1), (498, 498)
    else:
        raise ValueError("Incorrect difficulty level provided")


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 rrt_search.py maps/<mapname>.gif")
        sys.exit(1)

    difficulty = sys.argv[1]
    start, end = set_start_goal(difficulty)

    print(f"Running RRT on {difficulty}")
    print(f"Start: {start}")
    print(f"Goal : {end}")

    # Load binary map
    im = Image.open(difficulty).convert("1")
    width, height = im.size
    map_pixels = im.load()

    # --------------------------------------------------------
    # One visualization run
    # --------------------------------------------------------
    random.seed()  # fresh randomness
    t0 = time.time()
    success, tree_nodes, parents, final_path, iterations_used = rrt_search(
        map_pixels, width, height, start, end
    )
    t1 = time.time()

    if success:
        one_cost = path_cost(final_path)
        save_name = "rrt_result_" + difficulty.split("/")[-1].replace(".gif", ".png")
        visualize_rrt(save_name, difficulty, tree_nodes, parents, final_path, start, end)

        print("\nOne solved run:")
        print(f"  Success        : {success}")
        print(f"  Iterations     : {iterations_used}")
        print(f"  Path nodes     : {len(final_path)}")
        print(f"  Path cost      : {one_cost:.2f}")
        print(f"  Runtime (sec)  : {t1 - t0:.4f}")
        print(f"  Tree nodes     : {len(tree_nodes)}")
        print(f"  Saved image    : {save_name}")
    else:
        print("\nOne solved run:")
        print("  No path found in this trial.")
        print(f"  Iterations     : {iterations_used}")
        print(f"  Runtime (sec)  : {t1 - t0:.4f}")

    # --------------------------------------------------------
    # 100-run statistics
    # --------------------------------------------------------
    costs = []
    runtimes = []
    iterations_list = []
    tree_sizes = []
    success_count = 0

    print(f"\nRunning {RUNS} trials for statistics...")

    for run in range(RUNS):
        t0 = time.time()
        success, tree_nodes, parents, final_path, iterations_used = rrt_search(
            map_pixels, width, height, start, end
        )
        t1 = time.time()

        if success:
            success_count += 1
            costs.append(path_cost(final_path))
            runtimes.append(t1 - t0)
            iterations_list.append(iterations_used)
            tree_sizes.append(len(tree_nodes))

    print("\n" + "=" * 55)
    print("RRT STATISTICAL RESULTS")
    print("=" * 55)
    print(f"Map                : {difficulty}")
    print(f"Runs               : {RUNS}")
    print(f"Successes          : {success_count}")
    print(f"Failure count      : {RUNS - success_count}")
    print(f"Success rate       : {100.0 * success_count / RUNS:.2f}%")

    if success_count > 0:
        print("-" * 55)
        print(f"Path cost mean     : {mean(costs):.2f}")
        print(f"Path cost std dev  : {std_dev(costs):.2f}")
        print(f"Runtime mean (s)   : {mean(runtimes):.4f}")
        print(f"Runtime std dev(s) : {std_dev(runtimes):.4f}")
        print(f"Iterations mean    : {mean(iterations_list):.2f}")
        print(f"Iterations std dev : {std_dev(iterations_list):.2f}")
        print(f"Tree nodes mean    : {mean(tree_sizes):.2f}")
        print(f"Tree nodes std dev : {std_dev(tree_sizes):.2f}")
    else:
        print("-" * 55)
        print("No successful runs, so no path statistics are available.")

    print("=" * 55)