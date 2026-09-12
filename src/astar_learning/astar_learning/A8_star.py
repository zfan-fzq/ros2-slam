import heapq
from math import sqrt
import matplotlib.pyplot as plt
import math


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
        for neighbor, move_cost in get_neighbors(current, grid):
            if neighbor in closed:
                continue
            new_g = g_score[current] + move_cost
            if neighbor not in g_score or new_g < g_score[neighbor]:
                g_score[neighbor] = new_g
                parent[neighbor] = current
                f = new_g + heuristic(neighbor, goal)
                heapq.heappush(heap, (f, neighbor))
    return None  # Return None if no path is found


def heuristic(node, goal):
    dr = abs(node[0] - goal[0])
    dc = abs(node[1] - goal[1])

    diagonal_h = sqrt(2) * min(dr, dc)
    linear_h = max(dr, dc) - min(dr, dc)
    return diagonal_h + linear_h

def get_neighbors(node, grid):
    rows = len(grid)
    cols = len(grid[0])
    directions = [((0, 1), 1), 
                  ((1, 0), 1), 
                  ((0, -1), 1), 
                  ((-1, 0), 1),
                  ((1, 1),sqrt(2)), 
                  ((1, -1), sqrt(2)), 
                  ((-1, 1), sqrt(2)), 
                  ((-1, -1), sqrt(2))
                  ]  
    # Include diagonal movements with cost sqrt(2)
    neighbors = []
    for (dr,dc), move_cost in directions:
        nr = node[0] + dr
        nc = node[1] + dc
        if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:  # Check if the neighbor is within bounds and not an obstacle
            if dr != 0 and dc != 0:  # Diagonal movement
                if not grid[node[0]][nc] and not grid[nr][node[1]]:  # Check for corner cutting
                    neighbors.append(((nr, nc), move_cost))
            else:
                neighbors.append(((nr, nc), move_cost))
    return neighbors

def reconstruct_path(parent, goal):
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent[current]
    return path[::-1]   

