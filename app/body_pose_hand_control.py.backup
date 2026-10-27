#!/usr/bin/env python3
"""MediaPipe body-pose + hand-gesture control for Franka Panda."""
import argparse
import json
import math
from pathlib import Path
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from franka_msgs.action import Move

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "body_control.json"
POSE = mp.solutions.pose.PoseLandmark
HAND = mp.solutions.hands.HandLandmark

def clamp(value, lo, hi): return max(lo, min(hi, value))
def dist2(a, b): return math.hypot(a.x - b.x, a.y - b.y)
def vec3_sub(a, b): return (a.x - b.x, a.y - b.y, a.z - b.z)

class EMA3:
    def __init__(self, alpha): self.alpha, self.value = alpha, None
    def reset(self): self.value = None
    def update(self, v):
        if self.value is None: self.value = tuple(v)
        else:
            self.value = tuple(self.alpha*x + (1-self.alpha)*old for x,old in zip(v,self.value))
        return self.value

class PinchState:
    def __init__(self, cfg):
        self.close_ratio=float(cfg["pinch_close_ratio"]); self.open_ratio=float(cfg["pinch_open_ratio"])
        self.frames_needed=int(cfg["pinch_stable_frames"]); self.alpha=float(cfg["pinch_ema_alpha"])
        if self.close_ratio >= self.open_ratio: raise ValueError("pinch_close_ratio must be < pinch_open_ratio")
        self.reset()
    def reset(self): self.filtered=None; self.confirmed="NEUTRAL"; self.candidate="NEUTRAL"; self.frames=0
    def update(self, raw):
        self.filtered = raw if self.filtered is None else self.alpha*raw + (1-self.alpha)*self.filtered
        proposed = "CLOSE" if self.filtered <= self.close_ratio else ("OPEN" if self.filtered >= self.open_ratio else self.confirmed)
        if proposed == self.candidate: self.frames += 1
        else: self.candidate, self.frames = proposed, 1
        changed=False
        if self.candidate != "NEUTRAL" and self.frames >= self.frames_needed and self.candidate != self.confirmed:
            self.confirmed=self.candidate; changed=True
        return self.filtered, self.confirmed, changed

class RobotIO(Node):
    def __init__(self, cfg, dry_run=False):
        super().__init__("mediapipe_body_hand_control")
        self.cfg=cfg; self.dry_run=dry_run; self.gripper_busy=False
        self.twist_pub=self.create_publisher(TwistStamped, cfg["servo_twist_topic"], 10)
        self.gripper=ActionClient(self, Move, "/panda_gripper/move")
    def publish_twist(self, vx, vy, vz):
        if self.dry_run: return
        msg=TwistStamped(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=self.cfg["servo_frame_id"]
        msg.twist.linear.x=float(vx); msg.twist.linear.y=float(vy); msg.twist.linear.z=float(vz)
        self.twist_pub.publish(msg)
    def zero_twist(self): self.publish_twist(0.0,0.0,0.0)
    def gripper_command(self,label,width,speed):
        if self.dry_run:
            print(f"[DRY] {label}: width={width:.3f} m speed={speed:.3f} m/s"); return
        if self.gripper_busy: return
        if not self.gripper.server_is_ready() and not self.gripper.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning("/panda_gripper/move unavailable"); return
        goal=Move.Goal(); goal.width=float(width); goal.speed=float(speed); self.gripper_busy=True
        fut=self.gripper.send_goal_async(goal); fut.add_done_callback(self._goal_cb)
        self.get_logger().info(f"Gripper {label}: {width:.3f} m")
    def _goal_cb(self,future):
        try:
            handle=future.result()
            if not handle.accepted:
                self.gripper_busy=False; self.get_logger().error("Gripper goal rejected"); return
            rf=handle.get_result_async(); rf.add_done_callback(self._result_cb)
        except Exception as exc:
            self.gripper_busy=False; self.get_logger().error(f"Gripper send error: {exc}")
    def _result_cb(self,future):
        try:
            result=future.result().result
            self.get_logger().info("Gripper command succeeded" if result.success else f"Gripper command failed: {result.error}")
        except Exception as exc: self.get_logger().error(f"Gripper result error: {exc}")
        finally: self.gripper_busy=False

def load_config(path):
    with open(path,"r",encoding="utf-8") as f: return json.load(f)

def velocity_from_delta(delta,deadband,full_scale,vmax):
    if abs(delta) <= deadband: return 0.0
    mag=(abs(delta)-deadband)/max(full_scale-deadband,1e-6)
    return math.copysign(vmax*clamp(mag,0.0,1.0),delta)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--config",default=str(DEFAULT_CONFIG)); parser.add_argument("--dry-run",action="store_true"); args=parser.parse_args()
    cfg=load_config(args.config); rclpy.init(); robot=RobotIO(cfg,args.dry_run)
    holistic=mp.solutions.holistic.Holistic(static_image_mode=False,model_complexity=1,smooth_landmarks=True,refine_face_landmarks=False,
        min_detection_confidence=float(cfg["min_detection_confidence"]),min_tracking_confidence=float(cfg["min_tracking_confidence"]))
    drawing=mp.solutions.drawing_utils; styles=mp.solutions.drawing_styles
    cap=cv2.VideoCapture(int(cfg["camera_index"]))
    if not cap.isOpened(): raise RuntimeError(f"Cannot open camera index {cfg['camera_index']}")
    arm_master=False; gripper_enabled=False; neutral=None; body_filter=EMA3(float(cfg["body_ema_alpha"])); pinch=PinchState(cfg)
    pinch_ratio=None; pinch_label="NEUTRAL"; last_v=(0.0,0.0,0.0); deadman=False
    print("C=calibrate  A=arm master  G=gripper  R=reset  Q=quit")
    if args.dry_run: print("DRY RUN - no robot commands are sent")
    try:
        while rclpy.ok():
            rclpy.spin_once(robot,timeout_sec=0.001); ok,frame=cap.read()
            if not ok:
                robot.zero_twist()
                continue
            # Process the original camera image so anatomical LEFT/RIGHT labels remain consistent.
            # Mirror only the final display image for a natural user-facing preview.
            rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False; result=holistic.process(rgb); rgb.flags.writeable=True
            pose_ok=False; rel=None; deadman=False; vx=vy=vz=0.0
            if result.pose_landmarks and result.pose_world_landmarks:
                p2=result.pose_landmarks.landmark; pw=result.pose_world_landmarks.landmark
                ids=[POSE.RIGHT_SHOULDER.value,POSE.RIGHT_WRIST.value,POSE.LEFT_SHOULDER.value,POSE.LEFT_WRIST.value]
                if all(p2[i].visibility >= float(cfg["pose_visibility_threshold"]) for i in ids):
                    pose_ok=True; rel=body_filter.update(vec3_sub(pw[POSE.RIGHT_WRIST.value],pw[POSE.RIGHT_SHOULDER.value]))
                    deadman=p2[POSE.LEFT_WRIST.value].y < p2[POSE.LEFT_SHOULDER.value].y - float(cfg["deadman_vertical_margin"])
                    if neutral is not None:
                        dx,dy,dz=(rel[i]-neutral[i] for i in range(3)); db=float(cfg["body_deadband_m"]); fs=float(cfg["body_full_scale_m"]); vm=float(cfg["max_linear_speed_mps"])
                        bx=velocity_from_delta(dx,db,fs,vm); by=velocity_from_delta(dy,db,fs,vm); bz=velocity_from_delta(dz,db,fs,vm)
                        vx=float(cfg["robot_x_from_body_z_sign"])*bz; vy=float(cfg["robot_y_from_body_x_sign"])*bx; vz=float(cfg["robot_z_from_body_y_sign"])*by
                    drawing.draw_landmarks(frame,result.pose_landmarks,mp.solutions.holistic.POSE_CONNECTIONS,landmark_drawing_spec=styles.get_default_pose_landmarks_style())
            if result.right_hand_landmarks:
                h=result.right_hand_landmarks.landmark; palm=dist2(h[HAND.INDEX_FINGER_MCP.value],h[HAND.PINKY_MCP.value])
                if palm > 1e-6:
                    raw=dist2(h[HAND.THUMB_TIP.value],h[HAND.INDEX_FINGER_TIP.value])/palm; pinch_ratio,pinch_label,changed=pinch.update(raw)
                    if changed and gripper_enabled:
                        if pinch_label=="CLOSE": robot.gripper_command("CLOSE",cfg["close_width_m"],cfg["gripper_speed_mps"])
                        elif pinch_label=="OPEN": robot.gripper_command("OPEN",cfg["open_width_m"],cfg["gripper_speed_mps"])
                drawing.draw_landmarks(frame,result.right_hand_landmarks,mp.solutions.holistic.HAND_CONNECTIONS)
            if result.left_hand_landmarks: drawing.draw_landmarks(frame,result.left_hand_landmarks,mp.solutions.holistic.HAND_CONNECTIONS)
            arm_active=arm_master and neutral is not None and pose_ok and deadman
            if arm_active: robot.publish_twist(vx,vy,vz); last_v=(vx,vy,vz)
            else: robot.zero_twist(); last_v=(0.0,0.0,0.0)
            if cfg.get("mirror_image", True):
                frame=cv2.flip(frame,1)
            cv2.rectangle(frame,(10,10),(650,215),(0,0,0),-1)
            lines=[f"ARM MASTER: {'ON' if arm_master else 'OFF'}",f"DEADMAN: {'ON' if deadman else 'OFF'}",f"CALIBRATED: {'YES' if neutral is not None else 'NO'}",
                   f"GRIPPER: {'ON' if gripper_enabled else 'OFF'}  gesture={pinch_label}",f"Pinch ratio: {'--' if pinch_ratio is None else f'{pinch_ratio:.3f}'}",
                   f"Twist m/s: X {last_v[0]:+.3f}  Y {last_v[1]:+.3f}  Z {last_v[2]:+.3f}"]
            for i,line in enumerate(lines): cv2.putText(frame,line,(25,40+i*30),cv2.FONT_HERSHEY_SIMPLEX,0.66,(255,255,255),2)
            cv2.putText(frame,"C calibrate | A arm | G gripper | R reset | Q quit",(15,frame.shape[0]-20),cv2.FONT_HERSHEY_SIMPLEX,0.55,(255,255,255),2)
            cv2.imshow("Franka MediaPipe Body + Hand Control",frame); key=cv2.waitKey(1)&0xFF
            if key==ord('q'): break
            elif key==ord('a'): arm_master=not arm_master; print(f"Arm master: {'ON' if arm_master else 'OFF'}")
            elif key==ord('g'): gripper_enabled=not gripper_enabled; print(f"Gripper gestures: {'ON' if gripper_enabled else 'OFF'}")
            elif key==ord('c'):
                if rel is None: print("Cannot calibrate: right shoulder/wrist not reliably tracked")
                else: neutral=tuple(rel); print("Neutral calibrated:",neutral)
            elif key==ord('r'):
                arm_master=False; gripper_enabled=False; neutral=None; body_filter.reset(); pinch.reset(); robot.zero_twist(); print("Reset")
    finally:
        for _ in range(5): robot.zero_twist(); rclpy.spin_once(robot,timeout_sec=0.01)
        cap.release(); cv2.destroyAllWindows(); holistic.close(); robot.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
