import math

def occupancy_to_grid(data, width, height):
    grid = []
    for i in range(height):
        row = []
        for j in range(width):
            index = i * width + j
            if data[index] == 100:
                row.append(1)  # Obstacle
            elif data[index] == -1:
                row.append(-1)  # Unknown treated as obstacle
            else:
                row.append(0)  # Free space
        grid.append(row)
    return grid


def world_to_grid(x, y, origin_x, origin_y, resolution):

    col = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)

    return row, col


def grid_to_world(row, col, origin_x, origin_y, resolution):

    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution

    return x, y

def inflate_obstacles(grid, radius):
    rows = len(grid)
    cols = len(grid[0])
    inflated_grid = [row.copy() for row in grid]
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        nr = i + dr
                        nc = j + dc
                        if (dr ** 2 + dc ** 2 <= radius ** 2 and 
                        0 <= nr < rows and 0 <= nc < cols):
                            inflated_grid[nr][nc] = 1
    return inflated_grid
