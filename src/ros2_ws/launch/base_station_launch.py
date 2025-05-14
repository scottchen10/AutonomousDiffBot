import os

from sympy import true
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robotXacroName = "robot"
    packageName = "autonomous_diff_bot"
    # robotDescriptionPath = "hardware/robot_description.urdf.xacro"
    
    robotDescriptionPath = os.path.join(get_package_share_directory(packageName), "description/hardware/robot_description.urdf.xacro")
    print(robotDescriptionPath)
    robotDescription = xacro.process_file(robotDescriptionPath).toxml()
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments = {
            "gz_args": ["-r -v -v4 empty.sdf"],
            "on_exit_shutdown": "true"
        }.items()
    )
    
    spawnModelNodeGazebo = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments = [
            "-name", robotXacroName,
            "-topic", "robot_description"
        ],
        output = "screen"
    )
    
    nodeRobotStatePublisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters = [{
            "robot_description": robotDescription,
            "use_sim_time": True
        }]
    )
    
    bridge_params = os.path.join(
        get_package_share_directory(packageName),
        "description/gazebo/bridge_parameters.yaml"
    )
    
    start_gazebo_ros_bridge_cmd = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        arguments = [
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}"
        ],
        output = "screen"
    )
    
    return LaunchDescription([
        gazeboLaunch,
        spawnModelNodeGazebo,
        nodeRobotStatePublisher,
        start_gazebo_ros_bridge_cmd
    ])
    