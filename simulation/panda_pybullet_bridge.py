#!/usr/bin/env python3
"""ROS 2 MediaPipe TwistStamped + gripper Move action -> PyBullet Panda.

Simulation only: does not start franka_control, MoveIt Servo or hardware controllers.
Run using scripts/run_sim_panda.sh in a separate terminal, or run_sim_all.sh.
"""
import math
import time

import pybullet as pb
import pybullet_data
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from franka_msgs.action import Move


class PandaSimulator(Node):
    def __init__(self):
        super().__init__('panda_pybullet_simulator')
        self.physics_client = pb.connect(pb.GUI)
        if self.physics_client < 0:
            raise RuntimeError('Cannot create PyBullet GUI. Check that a desktop DISPLAY is available.')
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0.0, 0.0, -9.81)
        pb.setTimeStep(1.0 / 120.0)
        pb.loadURDF('plane.urdf')
        self.robot = pb.loadURDF('franka_panda/panda.urdf', useFixedBase=True)
        pb.resetDebugVisualizerCamera(cameraDistance=1.25, cameraYaw=42,
                                      cameraPitch=-32, cameraTargetPosition=[0.35, 0, 0.45])

        joint_ids = {}
        link_ids = {}
        for j in range(pb.getNumJoints(self.robot)):
            info = pb.getJointInfo(self.robot, j)
            joint_ids[info[1].decode('utf-8')] = j
            link_ids[info[12].decode('utf-8')] = j
        self.arm = [joint_ids[f'panda_joint{i}'] for i in range(1, 8)]
        self.fingers = [joint_ids[f'panda_finger_joint{i}'] for i in (1, 2)]
        self.ee = next((link_ids[name] for name in
                        ('panda_hand', 'panda_link8', 'panda_link7') if name in link_ids), None)
        if self.ee is None:
            raise RuntimeError('Panda URDF end-effector link not found')

        self.home = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
        for index, q in zip(self.arm, self.home):
            pb.resetJointState(self.robot, index, q)
        for index in self.fingers:
            pb.resetJointState(self.robot, index, 0.04)
        self.joint_target = self.home[:]
        pb.stepSimulation()
        link_state = pb.getLinkState(self.robot, self.ee, computeForwardKinematics=True)
        self.target_xyz = list(link_state[4])
        self.target_quat = list(link_state[5])
        self.width = 0.08
        self.velocity = [0.0, 0.0, 0.0]
        self.last_command_time = 0.0

        self.create_subscription(TwistStamped, '/servo_node/delta_twist_cmds', self.on_twist, 10)
        self.action_server = ActionServer(self, Move, '/panda_gripper/move', self.on_gripper)
        self.get_logger().info('SIMULATION ONLY: PyBullet Panda ready')
        self.get_logger().info('Listening: /servo_node/delta_twist_cmds and /panda_gripper/move')
        self.get_logger().info('No Franka IP, hardware connection or MoveIt Servo required')

    def on_twist(self, msg):
        # The MediaPipe app sends m/s, with a max default of 0.03 m/s.
        vals = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.velocity = [max(-0.05, min(0.05, float(x))) if math.isfinite(x) else 0.0
                         for x in vals]
        self.last_command_time = time.monotonic()

    def on_gripper(self, goal_handle):
        goal = goal_handle.request
        self.width = max(0.0, min(0.08, float(goal.width)))
        self.get_logger().info(f'Simulated gripper width: {self.width:.3f} m')
        goal_handle.succeed()
        result = Move.Result()
        result.success = True
        return result

    def step(self, dt):
        # Stop after timeout, even if camera process crashes or loses tracking.
        if time.monotonic() - self.last_command_time < 0.25:
            vx, vy, vz = self.velocity
        else:
            vx = vy = vz = 0.0

        if abs(vx) + abs(vy) + abs(vz) > 1e-8:
            self.target_xyz[0] = max(0.25, min(0.78, self.target_xyz[0] + vx * dt))
            self.target_xyz[1] = max(-0.48, min(0.48, self.target_xyz[1] + vy * dt))
            self.target_xyz[2] = max(0.14, min(0.92, self.target_xyz[2] + vz * dt))
            # Maintain the initial tool orientation while translating in XYZ.
            ik = pb.calculateInverseKinematics(
                self.robot, self.ee, self.target_xyz,
                targetOrientation=self.target_quat,
                maxNumIterations=90, residualThreshold=1e-4)
            if len(ik) >= 7 and all(math.isfinite(q) for q in ik[:7]):
                self.joint_target = list(ik[:7])

        pb.setJointMotorControlArray(
            self.robot, self.arm, pb.POSITION_CONTROL,
            targetPositions=self.joint_target,
            forces=[87.0] * 7)
        for index in self.fingers:
            pb.setJointMotorControl2(self.robot, index, pb.POSITION_CONTROL,
                                     targetPosition=self.width / 2.0, force=20.0)
        pb.stepSimulation()

    def close(self):
        self.action_server.destroy()
        self.destroy_node()
        if pb.isConnected():
            pb.disconnect()


def main():
    rclpy.init()
    sim = None
    try:
        sim = PandaSimulator()
        previous = time.monotonic()
        while rclpy.ok() and pb.isConnected():
            rclpy.spin_once(sim, timeout_sec=0.001)
            now = time.monotonic()
            sim.step(max(0.0, min(0.04, now - previous)))
            previous = now
            time.sleep(1.0 / 120.0)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if sim is not None:
            sim.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
