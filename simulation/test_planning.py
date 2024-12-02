import time
import numpy as np
import pybullet as p
from stretch import *
from utils.tools import *
from motion_planning import *
import random
import sys

# Initialize PyBullet GUI and set up the environment
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -9.81)

# Initialize the scene with the mobot and set up the mug in a fixed/random position
mobot, mug_id, obstacles = init_scene(p, mug_random=True)

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

            # Navigation goal condition: total_driving_distance <= 18
            if current_position[0] > 1.6 and current_position[1] > -0.35:
                print("Reached the goal region! Total driving distance: ", total_driving_distance)

    return True

def place_mug_in_drawer(mobot, target_position, constraint, max_steps=5000):
    """
    Align the robot and its end-effector with the drawer using base and arm controls.
    The robot considers both base and end-effector orientations while turning and moving.
    """
    for step in range(max_steps):
        # Get current positions and orientations
        base_position, _, base_orientation = get_robot_base_pose(p, mobot.robotId)
        ee_position, ee_orientation, _ = get_robot_ee_pose(p, mobot.robotId)
        
        # Calculate distances
        distance_to_drawer = np.linalg.norm(np.array(base_position[:2]) - np.array(target_position[:2]))
        ee_to_drawer_distance = np.linalg.norm(np.array(ee_position) - np.array(target_position))
        
        print(f"Step {step}: Base to Target Distance: {distance_to_drawer}, EE to Target Distance: {ee_to_drawer_distance}")
        
        # Stop if the end-effector is close enough
        mug_position = get_mug_pose(p)  # Get the current mug position
        
        # Check if the mug is already in the drawer
        if (motion_planning_test(p, mobot.robotId, target_position)):
            return True
        
        # Define End Effector Link Index as 18 
        EE_LINK_INDEX = 18

        # Obtain Joint Poses by Inverse Kinematics
        jointPoses = p.calculateInverseKinematics(mobot.robotId, EE_LINK_INDEX, target_position)

        # Get Number of Joints
        numJoints = p.getNumJoints(mobot.robotId)

        # Find joints that are not labelled as JOINT_FIXED
        joint_indices = []
        for j in range(numJoints):
            joint = p.getJointInfo(mobot.robotId, j)
            if joint[2] == p.JOINT_FIXED:
                # print("Fixed Joint: ", joint)
                continue
            joint_indices.append(joint[0])
        
        # print("New joint indices:", joint_indices)
        # print(jointPoses)

        # Update Joint movements with the joint poses computed from inv. kinematics
        for i in range(len(joint_indices)):
            p.setJointMotorControl2(bodyIndex=mobot.robotId,
                                    jointIndex=joint_indices[i],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    positionGain=0.1,
                                    velocityGain=1)

        # Give it some time to sleep for the motor control to take effect
        time.sleep(1 / 240)

    return False

def random_point_in_circle(radius):
    # Step 1: Random angle θ between 0 and 2π
    theta = random.uniform(0, 2 * math.pi)
    # Step 2: Random radius r' with uniform distribution inside the circle
    r_prime = math.sqrt(random.uniform(0, 1)) * radius
    # Step 3: Convert polar to Cartesian coordinates
    x = r_prime * math.cos(theta)
    y = r_prime * math.sin(theta)
    return (x, y)

TEST_CASES = [
    [0.27, -0.71, 0.92],
    [-1.70, -3.70, 0.46],
    [1.45, -1.68, 0.59]
]

# Main execution
if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise TypeError("Missing required argument: Test case.\nSpecify test case between 1 - 3, e.g. `python3 simulation/test_planning.py 1`")
    
    # Define environment variables
    grid_resolution = 0.3
    x_min, x_max = -5, 5
    y_min, y_max = -5, 5
    mug_id = 21
    
    test_case_id = int(sys.argv[1])
    target_position = TEST_CASES[test_case_id - 1]
    base_goal = target_position

    # Adjustment if the location is right under the cabinet, since the robot
    # isn't able to move to that position
    if 1.12 <= target_position[0] and target_position[0] <= 1.9:
        base_goal[0] = 1.81

    while True:
        # Find the optimal path from the robot to the target position
        link_pos, _, _ = get_robot_base_pose(p, mobot.robotId)
        base_x, base_y, _ = link_pos
        path = None
        for i in range(1000):
            rand_x, rand_y = random_point_in_circle(1)
            if base_goal[0] > 1: break
            try:
                path = find_path((base_x, base_y), (base_goal[0] + rand_x, base_goal[1] + rand_y), obstacles, 0.3, x_min, x_max, y_min, y_max)
                print("Path to the bedroom found:", path)
                break
            except ValueError:
                print("Failed to find path, step:", i + 1)
                continue

        if path:
            # Step 1: Attempt to move the robot from the start position to the goal position
            if navigate_to_goal(mobot, path):
                if place_mug_in_drawer(mobot, target_position, None, 500):
                    print("Object successfully located!")
                    print(f"Motion planning test completed for test case ID {test_case_id} to position {target_position}")
                    break
                else:
                    print("Failed to place EE in the target position.")
            else:
                print("Failed to navigate to the target position.")
        else:
            path = find_path((-1,0), (1.81,-4), obstacles, 0.3, x_min, x_max, y_min, y_max)
            print("Path to the bedroom found:", path)

            for i in range(1000):
                rand_x, rand_y = random_point_in_circle(0)
                try:                
                    path2 = find_path((1.81,-4), (base_goal[0] + rand_x, base_goal[1] + rand_y), obstacles, 0.05, x_min, x_max, y_min, y_max)
                    break
                except ValueError:
                    print("Failed to find path, step:", i + 1)
                    continue

            # Step 1: Attempt to move the robot from the start position to the goal position
            if navigate_to_goal(mobot, path) and navigate_to_goal(mobot, path2):
                if place_mug_in_drawer(mobot, target_position, None, 500):
                    print("Object successfully located!")
                    print(f"Motion planning test completed for test case ID {test_case_id} to position {target_position}")
                    break
                else:
                    print("Failed to place EE in the target position.")
            else:
                print("Failed to navigate to the target position.")
        
        # TODO: Remove this before submission. 
        #       Just so that it doesnt stop directly for video recording
        time.sleep(5)
    time.sleep(300)
