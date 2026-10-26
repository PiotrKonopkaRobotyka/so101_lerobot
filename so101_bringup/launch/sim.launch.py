"""Start the whole SO-101 simulation with one command.

All ROS nodes, including those from the included launch files, use the Gazebo
clock (use_sim_time); the upstream RViz node does not set it on its own.

Startup order:
1. Gazebo with the robot, robot_state_publisher and the /clock bridge. The robot is
   urdf/so101_sim.urdf.xacro: the upstream one with our controller configuration
   (config/so101_controllers.yaml, 100 Hz instead of 10 Hz; ADR 0004).
2. Controller spawner: waits for the controller_manager that runs inside Gazebo
   (gz_ros2_control plugin), activates the controllers and exits.
3. Only after the spawner has exited, so the trajectory controllers are active:
   MoveIt (move_group) and RViz (argument moveit, default true) and the
   hand teleoperation node with the webcam (argument hand, default false).

Hand teleoperation without MoveIt, the gripper only (stage 1) or also the arm (stage 2):
    ros2 launch so101_bringup sim.launch.py moveit:=false hand:=true
    ros2 launch so101_bringup sim.launch.py moveit:=false hand:=true arm:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("lerobot_description"), "launch", "so101_gazebo.launch.py"]
        ),
        launch_arguments={
            "model": PathJoinSubstitution([FindPackageShare("so101_bringup"), "urdf", "so101_sim.urdf.xacro"])
        }.items(),
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
        PathJoinSubstitution(
            [FindPackageShare("lerobot_moveit"), "launch", "so101_moveit.launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("moveit")),
    )

    hand = Node(
        package="so101_hand_ros",
        executable="hand_teleop",
        parameters=[
            PathJoinSubstitution([FindPackageShare("so101_hand_ros"), "config", "hand_teleop.yaml"]),
            {"arm": ParameterValue(LaunchConfiguration("arm"), value_type=bool)},
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("hand")),
    )

    start_after_controllers = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawner, on_exit=[moveit, hand])
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
            SetParameter(name="use_sim_time", value=True),
            gazebo,
            controller_spawner,
            start_after_controllers,
        ]
    )
