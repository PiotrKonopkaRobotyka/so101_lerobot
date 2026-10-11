# so101_lerobot

Scripts for the **SO-101** arm on **ROS 2 Jazzy** (Ubuntu 24.04), simulation only for now:

- `lerobot_mimic/` — the arm mirrors the operator's arm from a webcam (YOLO pose), see its README.
- `lerobot_teleop/run_teleop.py` — keyboard teleoperation of the simulated arm through LeRobot.

## Workspace

```
so101_ws/
├── .venv/                 # Python env (not in git)
└── src/
    ├── so101_lerobot/     # this repo
    ├── lerobot_ws/        # Pavankv92/lerobot_ws (Apache-2.0): description, controllers, MoveIt
    └── lerobot-ros/       # ycheng517/lerobot-ros (Apache-2.0): LeRobot ↔ ROS 2 bridge
```

Third-party repos are not copied here; they are pinned in `so101_lerobot.repos`:

```bash
cd so101_ws
vcs import src < src/so101_lerobot/so101_lerobot.repos
rosdep install --from-paths src --ignore-src -y
```

Built on top of a MoveIt 2 underlay (`moveit_ws`).

## Python environment

```bash
cd so101_ws
python3 -m venv .venv --system-site-packages
touch .venv/COLCON_IGNORE
.venv/bin/pip install -r src/so101_lerobot/requirements-lock.txt \
                      -r src/so101_lerobot/requirements.txt \
                      -c src/so101_lerobot/constraints.txt
.venv/bin/pip install --no-deps -e src/lerobot-ros/lerobot_robot_ros \
                                -e src/lerobot-ros/lerobot_teleoperator_devices
```

- `requirements.txt` — direct dependencies of these scripts.
- `requirements-lock.txt` — full pinned set (LeRobot, torch CUDA 12.8, …) without ROS packages.
- `constraints.txt` — keeps `opencv-python` at a version compatible with ROS 2 Jazzy.
- `setuptools<80` is required by colcon.

## Build and run

```bash
source <moveit_ws>/install/setup.bash
source .venv/bin/activate
python -m colcon build
```

Simulation (three terminals, each with `source install/setup.bash` and the venv active):

```bash
ros2 launch lerobot_description so101_gazebo.launch.py
ros2 launch lerobot_controller so101_controller.launch.py
ros2 launch lerobot_moveit so101_moveit.launch.py
```

Keyboard teleoperation (simulation running, venv active):

```bash
python src/so101_lerobot/lerobot_teleop/run_teleop.py
```

Model weights are read from `$DATA_DIR/models` (default `~/data/models`), e.g. `yolo26m-pose.pt`.
