# Architecture

```text
Webcam -> MediaPipe Holistic
        -> pose: right wrist - right shoulder -> calibration/smoothing/deadband/speed limit
        -> left wrist above shoulder -> deadman
        -> TwistStamped -> MoveIt Servo -> panda_arm_controller -> Panda
        -> right hand thumb-index pinch -> /panda_gripper/move -> Franka Hand
```

Arm motion requires tracked pose + calibration + keyboard arm master + left-hand deadman. Missing any gate sends zero velocity. Angular velocity is zero in this first version.
