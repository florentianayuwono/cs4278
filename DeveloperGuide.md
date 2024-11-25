Here’s an explanation of the keyboard input handling and how each key affects the robot, as well as how to use the `base_control()` and `arm_control()` functions.

### Keyboard Input Handling Keys

The code defines specific keys to control different aspects of the robot’s movement, arm motion, and gripper. Here’s a breakdown of the keys and their actions:

#### Movement (Arrow Keys)
- **Right Arrow** (`p.B3G_RIGHT_ARROW`): 
  - When pressed, sets `turn = -1`, making the robot turn right.
  - When released, resets `turn = 0`, stopping the turn.
- **Left Arrow** (`p.B3G_LEFT_ARROW`): 
  - When pressed, sets `turn = 1`, making the robot turn left.
  - When released, resets `turn = 0`, stopping the turn.
- **Up Arrow** (`p.B3G_UP_ARROW`): 
  - When pressed, sets `forward = 1`, making the robot move forward.
  - When released, resets `forward = 0`, stopping forward movement.
- **Down Arrow** (`p.B3G_DOWN_ARROW`): 
  - When pressed, sets `forward = -1`, making the robot move backward.
  - When released, resets `forward = 0`, stopping backward movement.

#### Arm Lift (Z and X Keys)
- **Z**: 
  - When pressed, sets `up = 1`, raising the robot’s arm.
  - When released, resets `up = 0`, stopping upward movement.
- **X**: 
  - When pressed, sets `up = -1`, lowering the arm.
  - When released, resets `up = 0`, stopping downward movement.

#### Arm Stretch (A and D Keys)
- **A**: 
  - When pressed, sets `stretch = -1`, retracting the arm.
  - When released, resets `stretch = 0`, stopping retraction.
- **D**: 
  - When pressed, sets `stretch = 1`, extending the arm.
  - When released, resets `stretch = 0`, stopping extension.

#### Arm Roll (R and F Keys)
- **R**: 
  - When pressed, sets `roll = 1`, causing the arm to roll clockwise.
  - When released, resets `roll = 0`, stopping the roll.
- **F**: 
  - When pressed, sets `roll = -1`, causing the arm to roll counterclockwise.
  - When released, resets `roll = 0`, stopping the roll.

#### Arm Yaw (Y and H Keys)
- **Y**: 
  - When pressed, sets `yaw = 1`, causing the arm to rotate clockwise (yaw).
  - When released, resets `yaw = 0`, stopping the yaw rotation.
- **H**: 
  - When pressed, sets `yaw = -1`, rotating the arm counterclockwise.
  - When released, resets `yaw = 0`, stopping the yaw rotation.

#### Gripper Control (Q and E Keys)
- **Q**: 
  - When pressed, sets `gripper_open = -1`, closing the gripper.
  - When released, resets `gripper_open = 0`, stopping the closing action.
- **E**: 
  - When pressed, sets `gripper_open = 1`, opening the gripper.
  - When released, resets `gripper_open = 0`, stopping the opening action.

### Using `base_control()` and `arm_control()` Functions

These two functions are used to apply the movement and arm manipulation commands to the robot based on the control variables.

#### `base_control(mobot, p, forward, turn)`
- **Purpose**: Controls the robot’s base movement based on `forward` and `turn` values.
- **Parameters**:
  - `mobot`: The robot object (initialized earlier as `mobot`).
  - `p`: The PyBullet module.
  - `forward`: Integer representing forward or backward movement (`1` for forward, `-1` for backward, `0` for no movement).
  - `turn`: Integer representing turning direction (`1` for left, `-1` for right, `0` for no turning).
- **Usage**: This function is called within the main loop to process base movement continuously, updating the robot’s position based on keyboard inputs.

#### `arm_control(mobot, p, up, stretch, roll, yaw)`
- **Purpose**: Controls the arm movement and orientation based on `up`, `stretch`, `roll`, and `yaw` values.
- **Parameters**:
  - `mobot`: The robot object.
  - `p`: The PyBullet module.
  - `up`: Integer representing lift direction (`1` for up, `-1` for down, `0` for no movement).
  - `stretch`: Integer representing arm stretching (`1` for extend, `-1` for retract, `0` for no movement).
  - `roll`: Integer representing rolling direction (`1` for clockwise, `-1` for counterclockwise, `0` for no roll).
  - `yaw`: Integer representing yaw direction (`1` for clockwise, `-1` for counterclockwise, `0` for no yaw rotation).
- **Usage**: Also called within the main loop, `arm_control()` updates the arm’s position and orientation, enabling complex manipulation actions like reaching and grasping.

Both functions ensure that the robot’s movement and arm actions are responsive to real-time keyboard inputs.