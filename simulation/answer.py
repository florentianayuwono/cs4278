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
mobot, mug_id, obstacles = init_scene(p, mug_random=False)
print(obstacles)
drawer_position = [3.8, 0, 0]  # Target region for navigation

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
    Plans and executes a motion to grasp the mug using inverse kinematics.
    """
    # Step 1: Get the mug's position
    mug_position = get_mug_pose(p, mug_id)
    print(f"Target mug position: {mug_position}")

    # Step 2: Reposition robot closer to the mug if needed
    robot_base_position, _, _ = get_robot_base_pose(p, mobot.robotId)
    distance_to_mug = np.linalg.norm(np.array(robot_base_position) - np.array(mug_position))
    print(f"Distance from robot base to mug: {distance_to_mug}")

    # Step 4: Calculate and apply inverse kinematics
    joint_positions = p.calculateInverseKinematics(
        bodyUniqueId=mobot.robotId,
        endEffectorLinkIndex=18,
        targetPosition=mug_position,
    )

    print("joint positions", joint_positions)
    for joint_idx, joint_angle in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyIndex=mobot.robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angle,
        )

    time.sleep(1)

    # Step 5: Validate proximity and grasp
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    distance_to_mug = np.linalg.norm(np.array(mug_position) - np.array(ee_position))
    print(f"Distance to mug after repositioning: {distance_to_mug}")

    if distance_to_mug > 0.2:
        print("Error: Mug is too far to grasp!")
        return False

    attach(mug_id, mobot.robotId, ee_link_index=18)
    print("Mug successfully grasped!")
    return True


def place_in_drawer(mobot, drawer_position):
    """
    Plans and executes motion to place the mug in the drawer using inverse kinematics.
    """
    # Step 1: Calculate joint angles for placing the mug
    joint_positions = p.calculateInverseKinematics(
        bodyUniqueId=mobot.robotId,
        endEffectorLinkIndex=18,  # Gripper link index
        targetPosition=drawer_position,
    )

    # Step 2: Apply the joint positions to move the arm
    for joint_idx, joint_angle in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyIndex=mobot.robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angle,
        )

    # Wait for motion to complete
    time.sleep(1)

    # Step 3: Validate proximity and release
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    distance_to_drawer = np.linalg.norm(np.array(drawer_position) - np.array(ee_position))
    print(f"Distance to drawer: {distance_to_drawer}")

    if distance_to_drawer > 0.1:
        print("Error: End-effector is too far from the drawer!")
        return False

    detach(mug_id)
    print("Mug successfully placed in the drawer!")
    return True


# Main execution
if __name__ == "__main__":
    # Step 1: Navigate the robot along the path to the drawer position
    # Define grid resolution and bounds
    grid_resolution = 0.3
    x_min, x_max = -5, 5
    y_min, y_max = -5, 5
    path = find_path((-1,0), (1.81,-4), obstacles, 0.3, x_min, x_max, y_min, y_max)

    print("Path found:", path)

    path2 = find_path((1.81,-4), (3.4, 0), obstacles, 0.05, x_min, x_max, y_min, y_max)

    print("Path found:", path2)

    # path3 = find_path((2,0), (3.8, 0.1), obstacles, 0.2, x_min, x_max, y_min, y_max)

    # print("Path found:", path3)

    navi_flag = navigate_to_goal(mobot, path)
    navi_flag = navigate_to_goal(mobot, path2)

    # Step 2: Pick up the mug if it is in place
    if navi_flag and not grasp_flag:
        reach_and_grasp(mobot, mug_id)
        grasp_flag = True
        print("Mug picked up!")

    # Step 3: Place the mug in the drawer
    if grasp_flag:
        place_in_drawer(mobot, drawer_position)
        print("Mug placed in the drawer!")
