def is_clear_path(grid_map, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0:
        step_x = 0
    else:
        step_x = 1 if dx > 0 else -1
    if dy == 0:
        step_y = 0
    else:
        step_y = 1 if dy > 0 else -1

    if abs(dx) >= abs(dy):
        slope = float(dy) / dx if dx != 0 else float('inf')
        y = y1
        for x in range(x1, x2 + step_x, step_x):
            if grid_map[y][x] == 1:  # Assuming 1 represents an obstacle
                return False
            y += slope * step_y
            if int(y) != y:  # Check if y is an integer, meaning it's not aligned with grid
                y += step_y * 0.5  # Move y to the next grid cell

    else:
        slope = float(dx) / dy if dy != 0 else float('inf')
        x = x1
        for y in range(y1, y2 + step_y, step_y):
            if grid_map[y][x] == 1:
                return False
            x += slope * step_x
            if int(x) != x:
                x += step_x * 0.5

    return True

