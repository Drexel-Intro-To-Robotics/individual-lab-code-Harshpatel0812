import sys
from PIL import Image
import queue as Queue

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)
end = (0, 0)
difficulty = ""

'''
These variables determine display color
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (180, 180, 180)
DARK_GRAY = (100, 100, 100)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

'''
These variables are determined and filled algorithmically
'''
path = []
expanded = {}
frontier = {}
open_set = Queue.PriorityQueue()
came_from = {}
cost_so_far = {}


def search(map_pixels, width, height):
    global path, expanded, frontier, came_from, cost_so_far, open_set

    open_set.put((0, start))
    came_from[start] = None
    cost_so_far[start] = 0
    frontier[start] = 0

    while not open_set.empty():
        current_cost, current = open_set.get()

        if current in expanded:
            continue

        expanded[current] = current_cost
        if current in frontier:
            del frontier[current]

        if current == end:
            break

        x, y = current

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy

            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue

            if not map_pixels[nx, ny]:
                continue

            neighbor = (nx, ny)

            if neighbor in expanded:
                continue

            new_cost = cost_so_far[current] + 1

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                frontier[neighbor] = new_cost
                open_set.put((new_cost, neighbor))

    if end in came_from:
        current = end
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()


def visualize_search(save_file):
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()

    for x, y in expanded.keys():
        pixel_access[x, y] = DARK_GRAY

    for x, y in frontier.keys():
        pixel_access[x, y] = LIGHT_GRAY

    for x, y in path:
        pixel_access[x, y] = PURPLE

    pixel_access[start[0], start[1]] = RED
    pixel_access[end[0], end[1]] = BLUE

    im.save(save_file)


if __name__ == "__main__":
    difficulty = str(sys.argv[1])
    print(f"Running {sys.argv[0]} with {difficulty} difficulty.")

    if difficulty == "maps/trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "maps/medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "maps/very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    elif difficulty == "maps/my_maze.gif":
        start = (0, 0)
        end = (500, 205)
    elif difficulty == "maps/my_maze2.gif":
        start = (0, 0)
        end = (599, 350)
    elif difficulty == "maps/super_hard.gif":
        start = (1, 1)
        end = (498, 498)
    else:
        assert False, "Incorrect difficulty level provided"

    im = Image.open(difficulty).convert("1")
    width, height = im.size

    search(im.load(), width, height)

    if len(path) > 0:
        print(f"Path found! Length: {len(path)} nodes")
        print(f"Final path cost: {len(path) - 1}")
    else:
        print("No path found!")

    save_name = "dijkstra_search_result_" + difficulty.split("/")[-1].replace(".gif", ".png")
    visualize_search(save_name)

    print(f"Saved result to: {save_name}")
    print(f"Expanded nodes: {len(expanded)}")
    print(f"Frontier nodes: {len(frontier)}")