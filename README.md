# Franka Panda MediaPipe Body-Pose + Hand-Gesture Control

This project combines **MediaPipe body pose**, **MediaPipe hand tracking**, **ROS 2 Humble**, **MoveIt Servo**, and the **Franka Hand**.

The intended real-robot setup is the working Panda configuration already validated in the lab:

- Ubuntu 22.04 / ROS 2 Humble
- Franka Panda at `172.16.0.2`
- Laptop Ethernet at `172.16.0.1/16`
- Panda workspace: `~/panda_ros2_ws`
- legacy libfranka runtime: `~/libfranka/build`
- MoveIt launch: `franka_moveit_config/moveit_real_arm_platform.launch.py`

## What the gestures do

### Right-arm body motion -> Panda Cartesian motion

After calibration, the **right wrist relative to the right shoulder** commands small Cartesian velocities:

- push right hand forward/back -> Panda X motion
- move right hand left/right -> Panda Y motion
- move right hand up/down -> Panda Z motion

The program publishes `geometry_msgs/msg/TwistStamped` to MoveIt Servo. It does **not** map camera coordinates directly to robot joint angles.

### Left hand = deadman safety gesture

The arm can move only when all of these are true:

1. operator pressed `A` to arm body control;
2. neutral body pose was calibrated with `C`;
3. **left wrist is raised above the left shoulder**;
4. MediaPipe is currently tracking the pose;
5. MoveIt Servo is running.

Lowering the left hand immediately commands zero Cartesian velocity.

### Right thumb + index finger -> gripper

- thumb/index pinch -> close to 40 mm
- thumb/index separated -> open to 80 mm

The gripper has its own keyboard enable (`G`) and is disabled when the program starts.

## Safety defaults

- arm master starts **OFF**
- gripper control starts **OFF**
- calibration required before arm motion
- left-hand deadman required continuously
- maximum Cartesian speed defaults to only **0.03 m/s**
- angular velocity is always zero in this version
- body-motion deadband prevents small tracking noise from moving the robot
- MoveIt Servo is used for collision checking, joint-limit handling, and singularity handling
- loss of pose tracking sends a zero twist

MoveIt Servo requires fresh timestamped Cartesian commands. The supplied Servo configuration publishes trajectories to `/panda_arm_controller/joint_trajectory`.

## Project layout

```text
Franka_MediaPipe_BodyPose_HandGesture_Control/
├── README.md
├── QUICK_START.txt
├── requirements.txt
├── app/body_pose_hand_control.py
├── config/body_control.json
├── scripts/
│   ├── install.sh
│   ├── source_env.sh
│   ├── start_robot.sh
│   ├── start_servo.sh
│   ├── enable_servo.sh
│   ├── stop_servo.sh
│   ├── check_system.sh
│   ├── run_dry_demo.sh
│   └── run_real_control.sh
├── ros2_ws/src/franka_body_servo_support/
│   ├── launch/servo_real_panda.launch.py
│   ├── config/servo_real_panda.yaml
│   ├── package.xml
│   └── setup.py
└── docs/
    ├── ARCHITECTURE.md
    ├── Franka_MediaPipe_BodyPose_Control_Report.tex
    └── Franka_MediaPipe_BodyPose_Control_Report.pdf
```

## 1. Extract

```bash
cd ~
unzip Franka_MediaPipe_BodyPose_HandGesture_Control.zip
cd Franka_MediaPipe_BodyPose_HandGesture_Control
```

## 2. One-time installation

Leave Conda if it is active:

```bash
conda deactivate
cd ~/Franka_MediaPipe_BodyPose_HandGesture_Control
bash scripts/install.sh
```

The installer creates a MediaPipe virtual environment and builds `~/panda_body_servo_ws`. It does not run `apt` automatically.

If MoveIt Servo is missing, check:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix moveit_servo
```

Resolve/install MoveIt Servo separately before real arm control.

## 3. Dry camera test first

```bash
cd ~/Franka_MediaPipe_BodyPose_HandGesture_Control
bash scripts/run_dry_demo.sh
```

Keyboard:

```text
C  calibrate neutral right-arm pose
A  arm/disarm body-motion commands
G  enable/disable gripper gestures
R  reset calibration and gesture state
Q  quit
```

Dry mode sends no robot commands.

## 4. Real robot startup

In Franka Desk: power on, unlock/recover, ensure no Reflex/error, activate FCI, keep the emergency stop accessible, and clear the workspace.

Check:

```bash
ping -c 3 172.16.0.2
```

### Terminal 1 - Panda + MoveIt + gripper

```bash
cd ~/Franka_MediaPipe_BodyPose_HandGesture_Control
bash scripts/start_robot.sh
```

Do not launch a second basic `franka_bringup` hardware stack at the same time.

### Terminal 2 - MoveIt Servo

```bash
cd ~/Franka_MediaPipe_BodyPose_HandGesture_Control
bash scripts/start_servo.sh
```

### Terminal 3 - verify and enable Servo

```bash
cd ~/Franka_MediaPipe_BodyPose_HandGesture_Control
bash scripts/check_system.sh
bash scripts/enable_servo.sh
```

You need `panda_arm_controller` and `joint_state_broadcaster` active. The Servo start service should report `success: true`.

### Terminal 4 - body + hand controller

```bash
cd ~/Franka_MediaPipe_BodyPose_HandGesture_Control
bash scripts/run_real_control.sh
```

Recommended sequence:

1. stand naturally in front of camera;
2. press `C` to calibrate;
3. verify displayed XYZ commands remain near zero while still;
4. press `G` and test the gripper first;
5. press `A` to enable arm master;
6. raise **left wrist above left shoulder** to activate the deadman;
7. move the right hand only a small amount;
8. lower the left hand to stop arm motion.

## 5. Default mapping

```text
right wrist toward/away camera -> Panda X velocity
right wrist left/right         -> Panda Y velocity
right wrist up/down            -> Panda Z velocity
left wrist above shoulder      -> arm deadman ENABLE
left wrist below shoulder      -> arm STOP
thumb-index pinch              -> gripper CLOSE
thumb-index apart              -> gripper OPEN
```

Axis signs can be changed in `config/body_control.json` after checking them in dry mode.

## 6. Important configuration

```json
{
  "max_linear_speed_mps": 0.03,
  "body_deadband_m": 0.04,
  "body_full_scale_m": 0.22,
  "robot_x_from_body_z_sign": -1.0,
  "robot_y_from_body_x_sign": 1.0,
  "robot_z_from_body_y_sign": -1.0,
  "open_width_m": 0.08,
  "close_width_m": 0.04
}
```

## 7. Servo configuration

```text
Move group:          panda_arm
Planning frame:      panda_link0
End-effector frame:  panda_link8
Input topic:         /servo_node/delta_twist_cmds
Input type:          speed_units
Output topic:        /panda_arm_controller/joint_trajectory
Output type:         trajectory_msgs/JointTrajectory
Collision checking:  enabled
```

The body node publishes a newly timestamped `TwistStamped` message for each camera frame. If tracking or a safety gate is lost, it publishes zero velocity.

## 8. Stop

Press `Q` in the MediaPipe window, then:

```bash
bash scripts/stop_servo.sh
```

Stop the Servo and MoveIt terminals with `Ctrl+C` and return the robot to a safe state in Franka Desk.

## Validation status

ROS 2 hardware communication, MoveIt real-hardware startup, the active Panda trajectory controller, approximately 30 Hz `/joint_states`, and Franka Hand open/close actions were already validated on the lab Panda. The **body-pose-to-MoveIt-Servo layer in this ZIP is the next experimental layer and must be validated in dry mode before enabling real arm motion.**
