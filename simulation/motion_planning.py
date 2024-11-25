import numpy as np
import heapq
import pybullet as p

x_min, x_max = -5, 5 # Adjust according to your environment's dimensions
y_min, y_max = -5, 5
grid_resolution = 0.1

def pos_to_idx(x, y, grid_resolution=0.1):
    """
    Convert a position (x, y) to grid indices (i, j).
    """
    if x < x_min or x > x_max or y < y_min or y > y_max:
        raise ValueError(f"Position ({x}, {y}) is out of bounds!")
    i = int((x - x_min) // grid_resolution)
    j = int((y - y_min) // grid_resolution)
    return (i, j)

def idx_to_pos(i, j, grid_resolution=0.1):
    """
    Convert grid indices (i, j) to a position (x, y).
    """
    x = x_min + i * grid_resolution
    y = y_min + j * grid_resolution
    return (x, y)

def build_occupancy_grid(collision_mapping, grid_resolution, x_min, x_max, y_min, y_max):
    """
    Build an occupancy grid using the collision mapping.

    Parameters:
    - collision_mapping: Dictionary with object names as keys and their AABBs as values.
    - grid_resolution: Resolution of the grid.
    - x_min, x_max, y_min, y_max: Bounds of the environment.

    Returns:
    - grid: 2D numpy array where 1 represents free space and 0 represents obstacles.
    """
    # Calculate grid dimensions
    nx = int((x_max - x_min) // grid_resolution) + 1
    ny = int((y_max - y_min) // grid_resolution) + 1

    # Initialize the grid with free space
    grid = np.ones((nx, ny), dtype=int)

    for obj_name, (aabb_min, aabb_max) in collision_mapping.items():
        # Convert AABB to grid indices
        min_idx = pos_to_idx(aabb_min[0], aabb_min[1], grid_resolution)
        max_idx = pos_to_idx(aabb_max[0], aabb_max[1], grid_resolution)

        # Mark the grid cells covered by this object's AABB as occupied
        for i in range(max(0, min_idx[0]), min(nx, max_idx[0] + 1)):
            for j in range(max(0, min_idx[1]), min(ny, max_idx[1] + 1)):
                grid[i, j] = 0  # Mark cell as occupied

    return grid

def heuristic(a, b, grid, obstacle_penalty_weight=1):
    """
    Combined heuristic for A* pathfinding:
    - Shortest path (Manhattan distance)
    - Penalty for proximity to obstacles.
    """
    # Manhattan distance to the goal
    distance_to_goal = abs(a[0] - b[0]) + abs(a[1] - b[1])

    # Proximity penalty (distance to obstacles)
    obstacle_proximity_penalty = 0
    neighbors = [
        (a[0] + dx, a[1] + dy)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
    ]
    for neighbor in neighbors:
        ni, nj = neighbor
        if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1] and grid[ni, nj] == 0:
            # Penalize for each obstacle nearby
            obstacle_proximity_penalty += 1

    # Combine heuristic with penalty
    return distance_to_goal + obstacle_penalty_weight * obstacle_proximity_penalty

def astar_grid(grid, start_idx, goal_idx, obstacle_penalty_weight=1):
    """
    Perform A* pathfinding on a 2D grid considering proximity to obstacles.
    """
    open_set = []
    heapq.heappush(open_set, (0, start_idx))
    came_from = {}

    g_score = {start_idx: 0}
    f_score = {start_idx: heuristic(start_idx, goal_idx, grid, obstacle_penalty_weight)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_idx:
            # Reconstruct the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_idx)
            path.reverse()
            return path

        # Define neighbor positions (4-connected grid)
        neighbors = [
            (current[0] + dx, current[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]

        for neighbor in neighbors:
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0], neighbor[1]] == 0:
                    continue  # Skip obstacles
                tentative_g_score = g_score[current] + 1  # Uniform cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(
                        neighbor, goal_idx, grid, obstacle_penalty_weight
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

def find_path(start_pos, goal_pos, collision_mapping, grid_resolution, x_min, x_max, y_min, y_max):
    """
    Find a path from start_pos to goal_pos using A* and the occupancy grid.
    """
    # Build the occupancy grid
    grid = build_occupancy_grid(collision_mapping, grid_resolution, x_min, x_max, y_min, y_max)

    # Convert start and goal positions to grid indices
    start_idx = pos_to_idx(*start_pos, grid_resolution)
    goal_idx = pos_to_idx(*goal_pos, grid_resolution)

    # Ensure start and goal indices are within grid bounds
    nx, ny = grid.shape
    if not (0 <= start_idx[0] < nx and 0 <= start_idx[1] < ny):
        raise ValueError("Start position is out of grid bounds.")
    if not (0 <= goal_idx[0] < nx and 0 <= goal_idx[1] < ny):
        raise ValueError("Goal position is out of grid bounds.")

    # Ensure start and goal positions are not inside obstacles
    if grid[start_idx[0], start_idx[1]] == 0:
        raise ValueError("Start position is inside an obstacle.")
    if grid[goal_idx[0], goal_idx[1]] == 0:
        raise ValueError("Goal position is inside an obstacle.")

    # Run the A* algorithm
    path_indices = astar_grid(grid, start_idx, goal_idx)
    if path_indices is None:
        print("No path found.")
        return None
    else:
        print(f"Path found with {len(path_indices)} steps.")

        # Convert path indices back to world positions
        path = [idx_to_pos(i, j, grid_resolution) for (i, j) in path_indices]

        # Draw the path in PyBullet
        for i in range(len(path) - 1):
            start_point = [path[i][0], path[i][1], 0.1]  # Start point (with slight Z offset to make it visible)
            end_point = [path[i + 1][0], path[i + 1][1], 0.1]  # End point (with slight Z offset)
            
            # Draw a line between consecutive waypoints
            p.addUserDebugLine(
                start_point, 
                end_point, 
                lineColorRGB=[1, 0, 0],  # Red color for the path
                lineWidth=3  # Optional: Line width
            )
        return path
