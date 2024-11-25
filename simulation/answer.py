import time
import numpy as np
import pybullet as p
from stretch import *
from utils.tools import *
from motion_planning import *

# Initialize PyBullet GUI and set up the environment
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -9.81)

# Initialize the scene with the mobot and set up the mug in a fixed position
mobot, obstacles = init_scene(p, mug_random=False)
print(obstacles)
drawer_position = [1.6, -0.35, 0]  # Target region for navigation

# Parameters for controlling the robot
forward = 0
turn = 0
up = 0
stretch = 0
gripper_open = 0
roll = 0
yaw = 0

# Tracking variables
total_driving_distance = 0
previous_position, _, _ = get_robot_base_pose(p, mobot.robotId)
navi_flag = False
grasp_flag = False

def navigate_to_goal(mobot, path):
    """
    Navigates the mobot along the specified path to the goal.
    Stops and returns True when the goal is reached.
    """
    global forward, turn, total_driving_distance, previous_position, previous_orientation

    for target_position in path:
        target_position = [target_position[0], target_position[1], 0]
        while True:
            time.sleep(1./240.)

            # Get the robot's current position and orientation
            current_position, _, euler_orientation = get_robot_base_pose(p, mobot.robotId)
            current_yaw = euler_orientation[2]  # Yaw angle for robot's current heading

            # Calculate distance to the target waypoint
            distance = np.linalg.norm(np.array(target_position) - np.array(current_position))

            # Check if the robot is close enough to the target waypoint
            if distance < 0.1:
                print(f"Reached waypoint {target_position}.")
                break  # Move to the next waypoint

            # Calculate the direction vector and angle to the target waypoint
            direction_to_target = np.array(target_position) - np.array(current_position)
            angle_to_target = np.arctan2(direction_to_target[1], direction_to_target[0])

            # Calculate the angle difference between the robot's current yaw and the target angle
            angle_diff = angle_to_target - current_yaw
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-π, π]

            # Decide whether to turn or move forward based on the angle difference
            if abs(angle_diff) > 0.1:  # Turn if the angle difference is significant
                turn = 0.5 if angle_diff > 0 else -0.5
                forward = 0
            else:
                turn = 0
                forward = 1  # Move forward when facing the target

            # Move the robot
            base_control(mobot, p, forward, turn)

            # Update driving distance
            total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
            previous_position = current_position


    print(f"Reached the goal region! Total driving distance: {total_driving_distance}")
    return True

def reach_and_grasp(mobot, mug_id):
    """
    Plans and executes a motion to grasp the mug.
    """
    # Target position for the mug
    mug_position = get_mug_pose(p, mug_id)

    # Motion planning to reach the mug
    print("Planning motion to reach the mug...")
    motion_planning_test(p, mobot.robotId, mug_position)

    # Move end-effector towards the mug for grasping
    attach(mug_id, mobot.robotId, ee_link_index=18)

def place_in_drawer(mobot, drawer_position):
    """
    Moves the robot's end-effector to place the mug in the drawer.
    """
    print("Moving towards drawer to place the mug...")
    motion_planning_test(p, mobot.robotId, drawer_position)

    # Detach the mug to place it down
    detach(attached_constraint)

# Main execution
if __name__ == "__main__":
    # Step 1: Navigate the robot along the path to the drawer position
    # Define grid resolution and bounds
    grid_resolution = 0.3
    x_min, x_max = -5, 5
    y_min, y_max = -5, 5
    path = find_path((-1,0), (1.81,-4), obstacles, 0.3, x_min, x_max, y_min, y_max)

    print("Path found:", path)

    path2 = find_path((1.81,-4), (3, 0), obstacles, 0.05, x_min, x_max, y_min, y_max)

    print("Path found:", path2)

    navi_flag = navigate_to_goal(mobot, path)
    navi_flag = navigate_to_goal(mobot, path2)

    # Step 2: Pick up the mug if it is in place
    if navi_flag and not grasp_flag:
        reach_and_grasp(mobot)
        grasp_flag = True
        print("Mug picked up!")

    # Step 3: Place the mug in the drawer
    if grasp_flag:
        place_in_drawer(mobot, drawer_position)
        print("Mug placed in the drawer!")
