import math
import os
import argparse
import sys
import time
import pybullet as p

import numpy as np


sys.path.append('./')

def init_scene(p, mug_random=False):
    root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../")

    collision_mapping = {} # to store the obstacles positions

    ################ Plane Environment
    plane_id = p.loadURDF(os.path.join(root_dir,"resource/urdf/plane.urdf"), [0, 0, 0])
    plane_texture_id = p.loadTexture(os.path.join(root_dir,"resource/texture/texture1.jpg"))
    p.changeVisualShape(0,-1,textureUniqueId=plane_texture_id)

    ################ Robot
    mobot_urdf_file = os.path.join(root_dir,"resource/urdf/stretch/stretch.urdf")
    mobot = Robot(pybullet_api=p, start_pos=[-0.8,0.0,0.03], urdf_file=mobot_urdf_file)

    for _ in range(30):
        p.stepSimulation()


    ################ table initialization
    table_height = 0.8
    table_width = 1.10 * 2.0
    table_depth = 1.0
    table_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[table_depth / 2.0, table_width / 2.0,
                                                                        table_height / 2.0])
    table_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[table_depth / 2.0, table_width / 2.0,
                                                                           table_height / 2.0])
    mass = 0
    table_id = p.createMultiBody(mass, baseCollisionShapeIndex=table_c, baseVisualShapeIndex=table_v,
                                          basePosition=(table_depth / 2.0 + 0.1, 0.05, table_height / 2.0))
    table_color = [128 / 255.0, 128 / 255.0, 128 / 255.0, 1.0]
    p.changeVisualShape(table_id, -1, rgbaColor=table_color)
    collision_mapping["table"] = p.getAABB(table_id)

    ################ wall initialization
    wall_height = 2.2
    wall_width = table_width + 4.0
    wall_depth = 0.02
    wall_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0, wall_height/2.0])
    wall_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0, wall_height/2.0])
    mass = 0
    wall_center_x = p.getAABB(table_id)[1][0] + wall_depth/2.0

    wall_v2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0-0.5, wall_height/2.0])
    wall_c2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0-0.5, wall_height/2.0])

    wall_id = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c2,\
                                             baseVisualShapeIndex=wall_v2,\
                                             basePosition=(wall_center_x, -1.4, wall_height/2.0))

    wall_id_back = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c,\
                                             baseVisualShapeIndex=wall_v,\
                                             basePosition=(wall_center_x-3.0, -1.9, wall_height/2.0))


    wall_id_front = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c,\
                                             baseVisualShapeIndex=wall_v,\
                                             basePosition=(wall_center_x+3.0, -1.9, wall_height/2.0))

    collision_mapping["wall"] = p.getAABB(wall_id)
    collision_mapping["wall back"] = p.getAABB(wall_id_back)
    collision_mapping["wall front"] = p.getAABB(wall_id_front)

    wall_width_left = 2.0
    wall_width_right = 6.0
    wall_depth = 0.02
    wall_v_left = p.createVisualShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_left/2.0, wall_height/2.0])
    wall_c_left = p.createCollisionShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_left/2.0, wall_height/2.0])
    wall_v_right = p.createVisualShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_right/2.0, wall_height/2.0])
    wall_c_right = p.createCollisionShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_right/2.0, wall_height/2.0])

    mass = 0
    wall_left_center_x = p.getAABB(table_id)[1][0] - wall_width_left/2.0
    wall_right_center_x = wall_left_center_x
    wall_left_center_y = p.getAABB(table_id)[0][1] - wall_depth/2.0
    wall_right_center_y = p.getAABB(table_id)[1][1] + wall_depth/2.0

    wall_left_id = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c_left,\
                                             baseVisualShapeIndex=wall_v_left,\
                                             basePosition=(wall_left_center_x-1.0, wall_left_center_y-0.92, wall_height/2.0),
                                             baseOrientation=p.getQuaternionFromEuler((0,0,np.pi/2.0)))
    wall_right_id = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c_right,\
                                             baseVisualShapeIndex=wall_v_right,\
                                             basePosition=(wall_right_center_x+1.0, wall_right_center_y, wall_height/2.0),
                                             baseOrientation=p.getQuaternionFromEuler((0,0,-np.pi/2.0)))

    wall_right_id2 = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c_right,\
                                             baseVisualShapeIndex=wall_v_right,\
                                             basePosition=(wall_right_center_x+1.0, wall_right_center_y-6.1, wall_height/2.0),
                                             baseOrientation=p.getQuaternionFromEuler((0,0,-np.pi/2.0)))

    wall_color = [204/255.0,242/255.0,255/255.0,1.0]
    p.changeVisualShape(wall_left_id,-1,rgbaColor=wall_color)
    p.changeVisualShape(wall_right_id,-1,rgbaColor=wall_color)
    p.changeVisualShape(wall_id,-1,rgbaColor=wall_color)

    collision_mapping["wall left"] = p.getAABB(wall_left_id)
    collision_mapping["wall right"] = p.getAABB(wall_right_id)
    collision_mapping["wall right 2"] = p.getAABB(wall_right_id2)

    urdf_dir = os.path.join(root_dir,"resource/urdf")

    ################ cabinet initialization
    # Cabinet dimensions (original size based on URDF)
    cabinet_height = 1  # Original height (scaled)
    cabinet_width = 1.5   # Original width (scaled)
    cabinet_depth = 0.5   # Original depth (scaled)

    # Scaling factor from URDF
    cabinet_scaling = 0.7  # Apply the same scaling factor as in the URDF

    # Apply scaling to dimensions
    scaled_height = cabinet_height * cabinet_scaling
    scaled_width = cabinet_width * cabinet_scaling
    scaled_depth = cabinet_depth * cabinet_scaling

    # Position of the cabinet (same as in the URDF initialization)
    table_z = p.getAABB(table_id)[1][2]  # Get the height of the table
    table_x = p.getAABB(table_id)[1][0]
    cabinet_position = [table_x + 0.2, -1.5, table_z + 0.5]

    # Cabinet orientation (same as in the URDF initialization)
    cabinet_orientation = p.getQuaternionFromEuler([0, 0, np.pi])  # Rotated 180 degrees around Z axis

    # Create the visual and collision shapes
    cabinet_v = p.createVisualShape(
        p.GEOM_BOX, 
        halfExtents=[scaled_depth / 2.0, scaled_width / 2.0, scaled_height / 2.0]
    )
    cabinet_c = p.createCollisionShape(
        p.GEOM_BOX, 
        halfExtents=[scaled_depth / 2.0, scaled_width / 2.0, scaled_height / 2.0]
    )

    # Create the multi-body object
    cabinet2_id = p.createMultiBody(
        baseMass=0,  # Static object, no mass
        baseCollisionShapeIndex=cabinet_c,
        baseVisualShapeIndex=cabinet_v,
        basePosition=cabinet_position,
        baseOrientation=cabinet_orientation
    )

    # Set the visual color (similar to the URDF)
    cabinet_color = [0.5, 0.5, 0.5, 1.0]  # Gray color
    p.changeVisualShape(cabinet2_id, -1, rgbaColor=cabinet_color)

    collision_mapping["cabinet2"] = p.getAABB(cabinet2_id)
    
    ################ fridge initialization
    # Fridge dimensions (original size based on URDF)
    fridge_height = 1.8  # Original height (scaled)
    fridge_width = 1   # Original width (scaled)
    fridge_depth = 0.8  # Original depth (scaled)

    # Scaling factor from URDF
    fridge_scaling = 1.0  # Apply the same scaling factor as in the URDF

    # Apply scaling to dimensions
    scaled_height = fridge_height * fridge_scaling
    scaled_width = fridge_width * fridge_scaling
    scaled_depth = fridge_depth * fridge_scaling

    # Position of the fridge (same as in the URDF initialization)
    fridge_position = [0.7, -3.1, 0.9]

    # Fridge orientation (same as in the URDF initialization, no rotation)
    fridge_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation

    # Create the visual and collision shapes
    fridge_v = p.createVisualShape(
        p.GEOM_BOX, 
        halfExtents=[scaled_depth / 2.0, scaled_width / 2.0, scaled_height / 2.0]
    )
    fridge_c = p.createCollisionShape(
        p.GEOM_BOX, 
        halfExtents=[scaled_depth / 2.0, scaled_width / 2.0, scaled_height / 2.0]
    )

    # Create the multi-body object
    fridge_id = p.createMultiBody(
        baseMass=0,  # Static object, no mass
        baseCollisionShapeIndex=fridge_c,
        baseVisualShapeIndex=fridge_v,
        basePosition=fridge_position,
        baseOrientation=fridge_orientation
    )

    # Set the fridge color (similar to the URDF)
    fridge_color = [1, 1, 1, 1.0]  # Light gray color
    p.changeVisualShape(fridge_id, -1, rgbaColor=fridge_color)

    collision_mapping["fridge"] = p.getAABB(fridge_id)

    ################ drawer initialization
    table_z = p.getAABB(table_id)[1][2]
    drawer_position = [3.84, 0.05,  0.42]
    drawer_scaling = 0.5
    drawer_orientation = p.getQuaternionFromEuler([0, 0, 0])
    drawer_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/drawers/d1/mobility.urdf"), \
                                    basePosition=drawer_position, \
                                    baseOrientation=drawer_orientation, \
                                    globalScaling=drawer_scaling, \
                                    useFixedBase=True)

    ################ bed initialization
    bed_height = 0.7#0.12 * 2.0
    bed_width = 1.8
    bed_depth = 2.2
    bed_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[bed_depth / 2.0, bed_width / 2.0,
                                                                        bed_height / 2.0])
    bed_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[bed_depth / 2.0, bed_width / 2.0,
                                                                           bed_height / 2.0])
    mass = 0
    bed_id = p.createMultiBody(mass, baseCollisionShapeIndex=bed_c, baseVisualShapeIndex=bed_v,
                                          basePosition=(bed_depth / 2.0 + 1.9, -1.45, bed_height / 2.0))
    bed_color = [128 / 255.0, 128 / 255.0, 128 / 255.0, 1.0]
    p.changeVisualShape(bed_id, -1, rgbaColor=bed_color)
    collision_mapping["bed"] = p.getAABB(bed_id)
    
    ################ microwave initialization
    table_z = p.getAABB(table_id)[1][2]
    microwave_position = [0.35, 0.72, table_z + 0.15]
    microwave_scaling = 0.4
    microwave_orientation = p.getQuaternionFromEuler([0, 0, 0])

    microwave_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/microwaves/7128/mobility.urdf"), \
                                       basePosition=microwave_position, \
                                       baseOrientation=microwave_orientation, \
                                       globalScaling=microwave_scaling, \
                                       useFixedBase=True)

    p.changeVisualShape(microwave_id, 1, rgbaColor=[0.2, 0.2, 0.2, 1], specularColor=[1., 1., 1.])
    p.changeVisualShape(microwave_id, 0, rgbaColor=[0.4, 0.4, 0.4, 1], specularColor=[1., 1., 1.])
    p.changeVisualShape(microwave_id, 2, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(microwave_id, 3, rgbaColor=[0.2, 0.2, 0.2, 1])
    p.resetJointState(microwave_id, 1, np.pi/2.0, 0.0)

    collision_mapping["microwave"] = p.getAABB(microwave_id)

    ################ box initialization
    box_position = [2.25, -3.5 , 0.2]
    box_scaling = 0.4
    box_orientation = p.getQuaternionFromEuler([0, 0.0, np.pi / 2.0 + 0])
    box_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/boxes/b4/mobility.urdf"),
                                 basePosition=box_position,
                                 baseOrientation=box_orientation,
                                 globalScaling=box_scaling,
                                 useFixedBase=False,
                                 flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

    numJoint = p.getNumJoints(box_id)
    box_AABB = p.getAABB(box_id, 0)
    box_height = box_AABB[1][2] - box_AABB[0][2]
    p.resetBasePositionAndOrientation(box_id, box_position, box_orientation)
    bbox = p.getAABB(box_id)
    bbox2 = p.getAABB(box_id, 0)
    p.resetJointState(box_id, 1, 0.9, 0.0)
    for ji in range(numJoint):
        p.setJointMotorControl2(box_id, ji, p.VELOCITY_CONTROL, force=0.5)

    collision_mapping["box"] = p.getAABB(box_id)

    ################ bottle initialization
    bottle_position = [drawer_position[0]+0.1, drawer_position[1]+0.1, table_z+0.49]
    bottle_scaling = 0.2
    bottle_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0.0, 0.0])
    bottle_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/bottles/b3/mobility.urdf"),
                                    basePosition=bottle_position,
                                    baseOrientation=bottle_orientation,
                                    useFixedBase=False,
                                    globalScaling=bottle_scaling,
                                    flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

    obj_friction_ceof = 2000.0
    p.changeDynamics(bottle_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(bottle_id, -1, rollingFriction=obj_friction_ceof)
    p.changeDynamics(bottle_id, -1, spinningFriction=obj_friction_ceof)

    p.changeDynamics(bottle_id, -1, mass=0.02)

    collision_mapping["bottle"] = p.getAABB(bottle_id)

    ################ bowl initialization
    bowl_position = [0.4, -0.6, table_z + 0.15]
    bowl_scaling = 0.2
    bowl_orientation = p.getQuaternionFromEuler([.0, 0.0, 0.0])
    bowl_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/bowls/b1/model.urdf"), \
                                 basePosition=bowl_position, \
                                 baseOrientation=bowl_orientation, \
                                 globalScaling=bowl_scaling, \
                                 useFixedBase=False, \
                                 flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    bowl_AABB = p.getAABB(bowl_id)
    bowl_height = bowl_AABB[1][2] - bowl_AABB[0][2]
    bowl_position[2] = table_z + bowl_height / 2.0
    p.resetBasePositionAndOrientation(bowl_id, bowl_position, bowl_orientation)
    obj_friction_ceof = 2000.0
    collision_mapping["bowl"] = p.getAABB(bowl_id)
    
    p.changeDynamics(bowl_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(bowl_id, -1, rollingFriction=obj_friction_ceof)
    p.changeDynamics(bowl_id, -1, spinningFriction=obj_friction_ceof)
    p.changeDynamics(bowl_id, -1, mass=0.2)

    ################ mug initialization
    mug_position = [0.25, -0.93, 1.53]
    mug_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0, np.pi - np.pi / 2.0])
    mug_scaling = 0.2
    mug_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/mugs/m2/model.urdf"),
                                 useFixedBase=False,
                                 globalScaling=mug_scaling,
                                 basePosition=mug_position,
                                 baseOrientation=mug_orientation)

    obj_friction_ceof = 4000.0
    p.changeDynamics(mug_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(mug_id, -1, rollingFriction=obj_friction_ceof)
    p.changeDynamics(mug_id, -1, spinningFriction=obj_friction_ceof)
    p.changeDynamics(mug_id, -1, mass=0.01)

    ################ trashbin initialization
    trashbin_position = [-1.1, -4.01, 0.48]
    trashbin_scaling = 1.0
    trashbin_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0.0, np.pi / 2.0])
    trashbin_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/trashbins/t2/model.urdf"), \
                                    useFixedBase=False,
                                    basePosition=trashbin_position, \
                                    baseOrientation=trashbin_orientation, \
                                    globalScaling=trashbin_scaling)
    p.changeVisualShape(trashbin_id, -1, rgbaColor=[200 / 255., 179 / 255., 179 / 255., 1])

    collision_mapping["trashbin"] = p.getAABB(trashbin_id)
    
    ################ pan initialization
    pan_position = [0.35, .2, table_z + 0.05]
    pan_scaling = 0.6
    pan_orientation = p.getQuaternionFromEuler([.0, 0.0, np.pi / 4.0])
    pan_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/pans/p1/model.urdf"), \
                                 basePosition=pan_position, \
                                 baseOrientation=pan_orientation, \
                                 globalScaling=pan_scaling, \
                                 useFixedBase=False, \
                                 flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    p.changeVisualShape(pan_id, 1, rgbaColor=[0.9, 0.9, 0.9, 1.0])
    p.changeDynamics(pan_id, -1, mass=0.001)

    ################ spatula initialization
    spatula_position = np.copy(np.array(pan_position))
    spatula_position[1] -= 0.3
    spatula_position[0] += 0.25
    spatula_position[2] += 0.1
    spatula_scaling = 0.4
    spatula_orientation = p.getQuaternionFromEuler([np.pi/2-np.pi/8.,0.,0.])
    spatula_id = p.loadURDF( os.path.join(urdf_dir, "obj_libs/spatula/model.urdf"),\
                                          basePosition=spatula_position,\
                                          baseOrientation=spatula_orientation,\
                                          globalScaling=spatula_scaling,\
                                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,useFixedBase=False)

    p.changeVisualShape(spatula_id, -1, rgbaColor=[0 / 255.0, 179 / 255., 179 / 255., 1])
    p.changeDynamics(spatula_id, -1, mass=0.01)

    ################ mug initialization
    mug_position = [drawer_position[0]-0.15, drawer_position[1], 1.5]
    mug_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0, np.pi + np.pi/2.0])
    if mug_random:
        mug_position[0] += np.random.uniform(-0.05,0.1)
        mug_position[1] += np.random.uniform(-0.1,0.1)
        mug_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0, np.pi + np.pi/2.0 + np.random.uniform(-np.pi/4.0,np.pi/4.0)])

    mug_scaling = 0.25
    mug_id = p.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/mugs/m1/model.urdf"),
                                    useFixedBase=False,
                                    globalScaling=mug_scaling,
                                    basePosition=mug_position,
                                    baseOrientation=mug_orientation)
    p.changeVisualShape(mug_id, -1, rgbaColor=[1.0,1.0,1.0,1])
    obj_friction_ceof = 4000.0
    p.changeDynamics(mug_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(mug_id, -1, mass=0.01)
    # mug id: 21

    # Round the position of obstacles to 2 digits
    for obj_name, (aabb_min, aabb_max) in collision_mapping.items():
        rounded_min = [math.floor(coord * 100) / 100 for coord in aabb_min]  # Round down
        rounded_max = [math.ceil(coord * 100) / 100 for coord in aabb_max]    # Round up
        collision_mapping[obj_name] = (tuple(rounded_min), tuple(rounded_max)) 

    # Draw the bounding box of obstacles
    for item in collision_mapping.values():
        draw_aabb(item[0], item[1])
    
    for _ in range(20):
        p.stepSimulation()

    p.changeVisualShape(mobot.robotId,0,rgbaColor=[1,0,0,1])
    p.changeVisualShape(mobot.robotId,1,rgbaColor=[0,1,0,1])

    p.setRealTimeSimulation(1)

    for j in range (p.getNumJoints(mobot.robotId)):
        print(p.getJointInfo(mobot.robotId,j))
      
    p.resetJointState(drawer_id, 5, 0.3)    # make the first drawer open

    for _ in range(30):
        p.stepSimulation()
    
    return mobot, mug_id, collision_mapping

def draw_aabb(aabb_min, aabb_max, color=[1, 0, 0], line_width=2):
    """
    Draws the AABB (Axis-Aligned Bounding Box) in PyBullet.
    
    Parameters:
    - aabb_min: Minimum corner of the AABB as a tuple (x_min, y_min, z_min).
    - aabb_max: Maximum corner of the AABB as a tuple (x_max, y_max, z_max).
    - color: Line color for the box, default is red ([1, 0, 0]).
    - line_width: Line width for the debug lines, default is 2.
    """
    # Define the 8 corners of the AABB
    corners = [
        [aabb_min[0], aabb_min[1], aabb_min[2]],  # Bottom-back-left
        [aabb_max[0], aabb_min[1], aabb_min[2]],  # Bottom-back-right
        [aabb_max[0], aabb_max[1], aabb_min[2]],  # Bottom-front-right
        [aabb_min[0], aabb_max[1], aabb_min[2]],  # Bottom-front-left
        [aabb_min[0], aabb_min[1], aabb_max[2]],  # Top-back-left
        [aabb_max[0], aabb_min[1], aabb_max[2]],  # Top-back-right
        [aabb_max[0], aabb_max[1], aabb_max[2]],  # Top-front-right
        [aabb_min[0], aabb_max[1], aabb_max[2]]   # Top-front-left
    ]
    
    # Define the edges of the box as pairs of corner indices
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
        (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
    ]
    
    # Draw each edge as a line
    for start, end in edges:
        p.addUserDebugLine(corners[start], corners[end], lineColorRGB=color, lineWidth=line_width)

def get_global_action_from_local(robot, delta_forward):
    # Get the current joint angle of joint 2 (rotation around z-axis)
    joint2_state = p.getJointState(robot, 3)
    current_yaw = joint2_state[0]  # Get the current rotation (yaw angle)
    
    # Calculate the delta in world coordinates using the yaw angle (rotation around z-axis)
    delta_x = delta_forward * np.cos(current_yaw)  # Change along world x-axis
    delta_y = delta_forward * np.sin(current_yaw)  # Change along world y-axis
    
    return delta_x, delta_y

def base_control(robot, p, forward=0, turn=0):
    x_forward, y_forward = get_global_action_from_local(robot.robotId, forward)
    p.setJointMotorControl2(robot.robotId,3,p.VELOCITY_CONTROL,targetVelocity=turn,force=1000)
    p.setJointMotorControl2(robot.robotId,1,p.VELOCITY_CONTROL,targetVelocity=x_forward,force=1000)
    p.setJointMotorControl2(robot.robotId,2,p.VELOCITY_CONTROL,targetVelocity=y_forward,force=1000)
    
def arm_control(robot, p, up=0, stretch=0, roll=0, yaw=0):
    # up and down
    p.setJointMotorControl2(robot.robotId,8,p.VELOCITY_CONTROL,targetVelocity=0.2*up,force=1000)

    # stretch and shrink
    p.setJointMotorControl2(robot.robotId,10,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robotId,11,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robotId,12,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robotId,13,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    
    # rotate
    p.setJointMotorControl2(robot.robotId,14,p.VELOCITY_CONTROL,targetVelocity=roll,force=1000)
    p.setJointMotorControl2(robot.robotId,16,p.VELOCITY_CONTROL,targetVelocity=yaw,force=1000)
    
def gripper_control(mobot, p, cmd=0):
    # 1 is open, 0 is close
    p.setJointMotorControl2(mobot.robotId,18,p.VELOCITY_CONTROL,targetVelocity=-cmd,force=1000)      # joint left finger
    p.setJointMotorControl2(mobot.robotId,19,p.VELOCITY_CONTROL,targetVelocity=cmd,force=1000)    # joint right gripper
    
class Robot:
    def __init__(self,pybullet_api,start_pos=[0.4,0.3,0.4],urdf_file=None,resource_dir=None,project_root_dir=None):
        self.p = p #pybullet_api

        self.gripperMaxForce = 1000.0
        self.armMaxForce = 200.0

        self.start_pos = start_pos
        self.camera_index = 13

        self.project_dir = project_root_dir
        self.resource_dir = resource_dir
        self.urdf_file = urdf_file
        self.robotId = self.p.loadURDF(self.urdf_file, self.start_pos, useFixedBase=True)
        self.p.resetBasePositionAndOrientation(self.robotId, self.start_pos, [0, 0, 0, 1])
        #  self.p.resetJointState(self.robotId, self.camera_index, -0.3)
        self.p.resetJointState(self.robotId, 4, 0.5)

    def get_observation(self):
        camera_link_pos = self.p.getLinkState(self.robotId,self.camera_index)[0]
        camera_link_ori = self.p.getLinkState(self.robotId,self.camera_index)[1]
        camera_link_rotmat = self.p.getMatrixFromQuaternion(camera_link_ori)
        camera_link_rotmat = np.array(camera_link_rotmat).reshape((3, 3))
        camera_link_pos -= 0.1*camera_link_rotmat[:,1]
        camera_target_link_pos = np.array(camera_link_pos)
        camera_target_link_pos = camera_target_link_pos - camera_link_rotmat[:,1]

        self.p.changeVisualShape(self.robotId,self.camera_index,rgbaColor=[0,0,1])

        camera_view_matrix = self.p.computeViewMatrix(cameraEyePosition=[camera_link_pos[0], camera_link_pos[1], camera_link_pos[2]],
                                         cameraTargetPosition=[camera_target_link_pos[0], camera_target_link_pos[1], camera_target_link_pos[2]],
                                         cameraUpVector=camera_link_rotmat[:,2])


        ratio = 1.5
        image_width = int(640 * ratio)
        image_height = 480
        #self.p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
        camera_proj_matrix = self.p.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10)
        #$initAxis(camera_link_pos, camera_link_ori)
        self.p.getCameraImage(width=image_width,
                                      height=image_height,
                                      viewMatrix = camera_view_matrix,
                                      projectionMatrix=camera_proj_matrix,
                                      renderer = p.ER_BULLET_HARDWARE_OPENGL)
    
    def get_position(self):
        return self.p.getBasePositionAndOrientation(self.robotId)[0]