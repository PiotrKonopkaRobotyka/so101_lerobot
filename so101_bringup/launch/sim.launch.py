"""Start the whole SO-101 simulation with one command.

All ROS nodes, including those from the included launch files, use the Gazebo
clock (use_sim_time); the upstream RViz node does not set it on its own.

Startup order:
1. Gazebo with the robot, robot_state_publisher and the /clock bridge.
2. Controller spawner: waits for the controller_manager that runs inside Gazebo
   (gz_ros2_control plugin), activates the controllers and exits.
3. MoveIt (move_group) and RViz, started only after the spawner has exited,
   so the trajectory controllers MoveIt connects to are already active.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
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
        )
    )

    start_moveit_after_controllers = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawner, on_exit=[moveit])
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            gazebo,
            controller_spawner,
            start_moveit_after_controllers,
        ]
    )
