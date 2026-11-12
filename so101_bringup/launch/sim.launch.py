"""Start the whole SO-101 simulation with one command.

All ROS nodes use the Gazebo clock (use_sim_time); the upstream RViz node does not set it on its own.

Startup order:
1. Gazebo with the robot, robot_state_publisher and the /clock bridge: the same pieces as the upstream
   so101_gazebo.launch.py, started here so that the GUI can take our view or stay off. The robot is
   urdf/so101_sim.urdf.xacro: the upstream one with our controller configuration (config/so101_controllers.yaml,
   100 Hz instead of 10 Hz; ADR 0004). The GUI shows the robot from its left side (config/gui_side.config);
   headless:=true starts the server only (scripted checks: the GUI needs OpenGL).
2. Controller spawner: waits for the controller_manager that runs inside Gazebo (gz_ros2_control plugin),
   activates the controllers and exits.
3. Only after the spawner has exited, so the trajectory controllers are active: MoveIt (move_group) and RViz
   (argument moveit, default true), the hand teleoperation node with the webcam (argument hand, default false)
   and, with the arm teleoperation, a second Gazebo window with the view from above (config/gui_top.config),
   oriented like the hand preview (ADR 0006).

Hand teleoperation without MoveIt, the gripper only (stage 1) or also the arm (stage 2, ADR 0006):
    ros2 launch so101_bringup sim.launch.py moveit:=false hand:=true
    ros2 launch so101_bringup sim.launch.py moveit:=false hand:=true arm:=true
Without a webcam and a screen (a recording instead, no windows): video:=<file> preview:=false headless:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    bringup = FindPackageShare("so101_bringup")
    gz_sim = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])

    # The URDF names its meshes package://lerobot_description/...: Gazebo looks for the package folder here
    meshes_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=str(Path(get_package_share_directory("lerobot_description")).parent)
    )
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([bringup, "urdf", "so101_sim.urdf.xacro"])]), value_type=str
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    gui_side = PathJoinSubstitution([bringup, "config", "gui_side.config"])
    gazebo = IncludeLaunchDescription(
        gz_sim,
        launch_arguments={"gz_args": [" -v 4 -r empty.sdf --gui-config ", gui_side]}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        gz_sim, launch_arguments={"gz_args": " -s -v 4 -r empty.sdf"}.items(), condition=IfCondition(headless)
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "so101"],
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    moveit = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("lerobot_moveit"), "launch", "so101_moveit.launch.py"]),
        condition=IfCondition(LaunchConfiguration("moveit")),
    )

    hand = Node(
        package="so101_hand_ros",
        executable="hand_teleop",
        parameters=[
            PathJoinSubstitution([FindPackageShare("so101_hand_ros"), "config", "hand_teleop.yaml"]),
            {
                "arm": ParameterValue(LaunchConfiguration("arm"), value_type=bool),
                "video_path": ParameterValue(LaunchConfiguration("video"), value_type=str),
                "preview": ParameterValue(LaunchConfiguration("preview"), value_type=bool),
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("hand")),
    )

    # A second GUI client of the running Gazebo server: the view from above, for aiming with the hand
    top_view = ExecuteProcess(
        cmd=["gz", "sim", "-g", "--gui-config", PathJoinSubstitution([bringup, "config", "gui_top.config"])],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("arm"), "' == 'true' and '", headless, "' != 'true'"])
        ),
    )

    start_after_controllers = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawner, on_exit=[moveit, hand, top_view])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("moveit", default_value="true", description="Start MoveIt and RViz"),
            DeclareLaunchArgument(
                "hand", default_value="false", description="Start the hand teleoperation node (webcam)"
            ),
            DeclareLaunchArgument(
                "arm", default_value="false", description="With hand:=true, the hand also moves the arm (stage 2)"
            ),
            DeclareLaunchArgument("video", default_value="", description="Hand node: a video file, not the webcam"),
            DeclareLaunchArgument("preview", default_value="true", description="Hand node: show the preview window"),
            DeclareLaunchArgument("headless", default_value="false", description="Gazebo server only, no windows"),
            SetParameter(name="use_sim_time", value=True),
            meshes_path,
            robot_state_publisher,
            gazebo,
            gazebo_headless,
            spawn_robot,
            clock_bridge,
            controller_spawner,
            start_after_controllers,
        ]
    )
