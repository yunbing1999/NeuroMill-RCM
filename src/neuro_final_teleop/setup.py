from glob import glob
import os

from setuptools import find_packages, setup


package_name = "neuro_final_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "assets"), glob("neuro_final_teleop/assets/*.jpg")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "scipy",
        "pygame",
        "dualsense-controller",
        "xarm-python-sdk",
        "PyQt5",
        "pyqtgraph",
    ],
    zip_safe=True,
    maintainer="NeuroFinal",
    maintainer_email="igabilayeff@gmail.com",
    description=(
        "Clean NeuroFinal teleoperation package for xArm7 spinal drilling research: "
        "PS5 teleop, fixed-point control, force haptics, GUI, and session recording."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "neuro_final_teleop = neuro_final_teleop.neuro_final_teleop:main",
            "ft_bridge = neuro_final_teleop.nodes.ft_bridge:main",
            "joint_state_bridge = neuro_final_teleop.nodes.joint_state_bridge:main",
            "session_gui = neuro_final_teleop.nodes.session_gui:main",
        ],
    },
)
