import sys
from lerobot.scripts.lerobot_teleoperate import main

# Import the plugins so that their @RobotConfig.register_subclass decorators (from ycheng517/lerobot-ros) run
import lerobot_robot_ros
import lerobot_teleoperator_devices

if __name__ == "__main__":
    # Override the command-line arguments (same as typing them after lerobot-teleoperate in bash)
    sys.argv = [
        "lerobot-teleoperate",
        "--robot.type=so101_ros",
        "--robot.id=my_awesome_follower_arm",
        "--teleop.type=keyboard_joint",
        "--teleop.id=my_awesome_leader_arm",
        "--display_data=true"
    ]
    main()
