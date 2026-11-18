from setuptools import setup

package_name = "so101_hand_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/hand_teleop.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Piotr Konopka",
    maintainer_email="piotr.konopka.robotyka@gmail.com",
    description="ROS 2 node: the operator's hand seen by the webcam drives the simulated SO-101 gripper and arm",
    license="Apache-2.0",
    entry_points={"console_scripts": ["hand_teleop = so101_hand_ros.hand_teleop:main"]},
)
