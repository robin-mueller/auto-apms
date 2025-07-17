from setuptools import find_packages, setup

package_name = "auto_apms_ros2cli"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "argcomplete", "PyYAML"],
    zip_safe=True,
    maintainer="Robin Müller",
    maintainer_email="mue.robin@icloud.com",
    description="AutoAPMS related extensions for the ROS 2 CLI",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "ros2cli.command": [
            "behavior = auto_apms_ros2cli.command.behavior:BehaviorCommand",
        ],
        "auto_apms_ros2cli.verb": [
            "list = auto_apms_ros2cli.verb.list:ListVerb",
            "show = auto_apms_ros2cli.verb.show:ShowVerb",
            "run = auto_apms_ros2cli.verb.run:RunVerb",
            "send = auto_apms_ros2cli.verb.send:SendVerb",
        ],
    },
)
