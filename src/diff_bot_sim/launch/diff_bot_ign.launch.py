#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    diff_bot_path = get_package_share_directory("diff_bot_sim")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', join(diff_bot_path, 'urdf/diff_bot.xacro'),
                     ' sim_ign:=', "true"
                    ])}],
        remappings=[
            ('/joint_states', 'diff_bot/joint_states'),
        ]
    )

    # Launch rviz with the diff_bot display config
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", join(diff_bot_path, "rviz", "default.rviz")]
    )

    # Define Gazebo simulation launch
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    world_file = join(diff_bot_path, "worlds", "empty.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "diff_bot",
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0"
        ]
    )

    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/model/diff_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"

        ],
        remappings=[
            ('/world/empty/model/diff_bot/joint_state', 'diff_bot/joint_states'),
            ('/cmd_vel', 'diff_bot/cmd_vel'),
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
        gz_sim,
        gz_spawn_entity,
        gz_ros_bridge
    ])