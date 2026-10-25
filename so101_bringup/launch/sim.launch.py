"""Start the whole SO-101 simulation with one command.

All ROS nodes, including those from the included launch files, use the Gazebo
clock (use_sim_time); the upstream RViz node does not set it on its own.

Startup order:
1. Gazebo with the robot, robot_state_publisher and the /clock bridge.
2. Controller spawner: waits for the controller_manager that runs inside Gazebo
   (gz_ros2_control plugin), activates the controllers and exits.
3. Only after the spawner has exited, so the trajectory controllers are active:
   MoveIt (move_group) and RViz (argument moveit, default true) and the
   hand-tracking gripper node with the webcam (argument hand, default false).

Hand teleoperation of the gripper (stage 1) without MoveIt:
    ros2 launch so101_bringup sim.launch.py moveit:=false hand:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("lerobot_description"), "launch", "so101_gazebo.launch.py"]
        )
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
        executable="hand_gripper",
        parameters=[
            PathJoinSubstitution([FindPackageShare("so101_hand_ros"), "config", "hand_gripper.yaml"])
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
                "hand", default_value="false", description="Start the hand-tracking gripper node (webcam)"
            ),
            SetParameter(name="use_sim_time", value=True),
            gazebo,
            controller_spawner,
            start_after_controllers,
        ]
    )
