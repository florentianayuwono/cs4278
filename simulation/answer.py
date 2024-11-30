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

def align_with_mug(mobot, mug_id, max_steps=5000):
    """
    Align the robot and its end-effector with the mug using base and arm controls.
    The robot considers both base and end-effector orientations while turning.
    """
    for step in range(max_steps):
        base_position, _, base_orientation = get_robot_base_pose(p, mobot.robotId)
        ee_position, ee_orientation, _ = get_robot_ee_pose(p, mobot.robotId)
        mug_position = get_mug_pose(p, mug_id)
        
        # Calculate distances
        distance_to_mug = np.linalg.norm(np.array(base_position[:2]) - np.array(mug_position[:2]))
        ee_to_mug_distance = np.linalg.norm(np.array(ee_position) - np.array(mug_position))
        
        print(f"Step {step}: Base to Mug Distance: {distance_to_mug}, EE to Mug Distance: {ee_to_mug_distance}")
        
        # Stop if the end-effector is close enough
        if ee_to_mug_distance < 0.2:
            print("Aligned with the mug!")
            return True
        
        # Define End Effector Link Index as 18 
        EE_LINK_INDEX = 18

        # Obtain Joint Poses by Inverse Kinematics
        jointPoses = p.calculateInverseKinematics(mobot.robotId, EE_LINK_INDEX, mug_position)

        # Get Number of Joints
        numJoints = p.getNumJoints(mobot.robotId)

        # Find joints that are not labelled as JOINT_FIXED
        joint_indices = []
        for j in range(numJoints):
            joint = p.getJointInfo(mobot.robotId, j)
            if joint[2] == p.JOINT_FIXED:
                print("Fixed Joint: ", joint)
                continue
            joint_indices.append(joint[0])
        
        print("New joint indices:", joint_indices)
        print(jointPoses)

        # Update Joint movements with the joint poses computed from inv. kinematics
        for i in range(len(joint_indices)):
            p.setJointMotorControl2(bodyIndex=mobot.robotId,
                                    jointIndex=joint_indices[i],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    positionGain=0.01,
                                    velocityGain=0.1)

        # Give it some time to sleep for the motor control to take effect
        time.sleep(1. / 240.)

    return False

def place_mug_in_drawer(mobot, drawer_position, constraint, max_steps=5000):
    """
    Align the robot and its end-effector with the drawer using base and arm controls.
    The robot considers both base and end-effector orientations while turning and moving.
    """
    for step in range(max_steps):
        # Get current positions and orientations
        base_position, _, base_orientation = get_robot_base_pose(p, mobot.robotId)
        ee_position, ee_orientation, _ = get_robot_ee_pose(p, mobot.robotId)
        
        # Calculate distances
        distance_to_drawer = np.linalg.norm(np.array(base_position[:2]) - np.array(drawer_position[:2]))
        ee_to_drawer_distance = np.linalg.norm(np.array(ee_position) - np.array(drawer_position))
        
        print(f"Step {step}: Base to Drawer Distance: {distance_to_drawer}, EE to Drawer Distance: {ee_to_drawer_distance}")
        
        # Stop if the end-effector is close enough
        mug_position = get_mug_pose(p)  # Get the current mug position
        
        # Check if the mug is already in the drawer
        if (3.3 < mug_position[0] < 3.5 and
            -0.17 < mug_position[1] < 0.25 and
            0.71 < mug_position[2] < 0.75):
            detach(constraint)
            print("Mug is in the drawer!")
            return True  # Success condition
        
        # Define End Effector Link Index as 18 
        EE_LINK_INDEX = 18

        # Obtain Joint Poses by Inverse Kinematics
        jointPoses = p.calculateInverseKinematics(mobot.robotId, EE_LINK_INDEX, drawer_position)

        # Get Number of Joints
        numJoints = p.getNumJoints(mobot.robotId)

        # Find joints that are not labelled as JOINT_FIXED
        joint_indices = []
        for j in range(numJoints):
            joint = p.getJointInfo(mobot.robotId, j)
            if joint[2] == p.JOINT_FIXED:
                print("Fixed Joint: ", joint)
                continue
            joint_indices.append(joint[0])
        
        print("New joint indices:", joint_indices)
        print(jointPoses)

        # Update Joint movements with the joint poses computed from inv. kinematics
        for i in range(len(joint_indices)):
            p.setJointMotorControl2(bodyIndex=mobot.robotId,
                                    jointIndex=joint_indices[i],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    positionGain=0.03,
                                    velocityGain=0.5)

        # Give it some time to sleep for the motor control to take effect
        time.sleep(1. / 240.)

    return False

# Main execution
if __name__ == "__main__":
    # Define environment variables
    grid_resolution = 0.3
    x_min, x_max = -5, 5
    y_min, y_max = -5, 5
    mug_id = 21
    drawer_position = [3.4, 0.2, 0.73]

    # Find the optimal path from the robot to the drawer
    path = find_path((-1,0), (1.81,-4), obstacles, 0.3, x_min, x_max, y_min, y_max)
    print("Path to the bedroom found:", path)

    path2 = find_path((1.81,-4), (3.5, 0.5), obstacles, 0.05, x_min, x_max, y_min, y_max)
    print("Path to the drawer found:", path2)

    # Step 1: Attempt to move the robot from the start position to the goal position
    if navigate_to_goal(mobot, path) and navigate_to_goal(mobot, path2):
        # Step 2: Attempt to pick up the mug
        if align_with_mug(mobot, mug_id):
            constraint = attach(mug_id, mobot.robotId, ee_link_index=18)  # Grasp the mug
            # Step 3: Attempt to place the mug in the drawer
            if place_mug_in_drawer(mobot, drawer_position, constraint):
                print("Mug successfully placed in the drawer!")
            else:
                print("Failed to place mug in the drawer.")
        else:
            print("Failed to pick up the mug.")
    else:
        print("Failed to navigate to the drawer position.")
    
    # TODO: Remove this before submission. 
    #       Just so that it doesnt stop directly for video recording
    time.sleep(300)
