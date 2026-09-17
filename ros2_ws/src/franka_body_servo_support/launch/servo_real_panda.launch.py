import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def load_yaml(package_name, file_path):
    path=os.path.join(get_package_share_directory(package_name),file_path)
    with open(path,'r') as f: return yaml.safe_load(f)

def generate_launch_description():
    robot_ip=LaunchConfiguration('robot_ip')
    desc_xacro=os.path.join(get_package_share_directory('franka_description'),'robots','panda_arm_platform.urdf.xacro')
    robot_description={'robot_description':Command([FindExecutable(name='xacro'),' ',desc_xacro,' hand:=true',' camera:=false',' camera_model:=blackfly_s',' robot_ip:=',robot_ip,' use_fake_hardware:=false',' fake_sensor_commands:=false'])}
    semantic_xacro=os.path.join(get_package_share_directory('franka_moveit_config'),'srdf','panda_arm_platform.srdf.xacro')
    robot_description_semantic={'robot_description_semantic':Command([FindExecutable(name='xacro'),' ',semantic_xacro,' hand:=true',' camera:=false'])}
    kinematics={'robot_description_kinematics': load_yaml('franka_moveit_config','config/kinematics.yaml')}
    servo_params={'moveit_servo':load_yaml('franka_body_servo_support','config/servo_real_panda.yaml')}
    servo_node=Node(package='moveit_servo',executable='servo_node_main',name='servo_node',output='screen',parameters=[servo_params,robot_description,robot_description_semantic,kinematics])
    return LaunchDescription([DeclareLaunchArgument('robot_ip',default_value='172.16.0.2'),servo_node])
