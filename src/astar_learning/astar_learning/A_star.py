import heapq

def astar(grid, start, goal):
    g_score = {start: 0}
    parent = {start: None}
    closed = set() 
    heap = []
    f = heuristic(start, goal)
    heapq.heappush(heap, (f, start))
    while heap:
        current_f, current = heapq.heappop(heap)
        if current == goal:
            return reconstruct_path(parent, goal)
        closed.add(current)
        for neighbor in get_neighbors(current, grid):
            if neighbor in closed:
                continue
            new_g = g_score[current] + 1 
            if neighbor not in g_score or new_g < g_score[neighbor]:
                g_score[neighbor] = new_g
                parent[neighbor] = current
                f = new_g + heuristic(neighbor, goal)
                heapq.heappush(heap, (f, neighbor))
    return None  # Return None if no path is found


def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def get_neighbors(node, grid):
    rows = len(grid)
    cols = len(grid[0])
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] 
    neighbors = []
    for direction in directions:
        nr = node[0] + direction[0]
        nc = node[1] + direction[1]
        if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:  # Check if the neighbor is within bounds and not an obstacle
            neighbors.append((nr, nc))
    return neighbors

def reconstruct_path(parent, goal):
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent[current]
    return path[::-1]  

