from setuptools import find_packages, setup

package_name = "auto_apms_ros2behavior"

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
            "behavior = auto_apms_ros2behavior.command.behavior:BehaviorCommand",
        ],
        "auto_apms_ros2behavior.verb": [
            "list = auto_apms_ros2behavior.verb.list:ListVerb",
            "show = auto_apms_ros2behavior.verb.show:ShowVerb",
            "run = auto_apms_ros2behavior.verb.run:RunVerb",
            "send = auto_apms_ros2behavior.verb.send:SendVerb",
            "node = auto_apms_ros2behavior.verb.node:NodeVerb",
        ],
        "auto_apms_ros2behavior.verb.node": [
            "plugins = auto_apms_ros2behavior.verb.node.plugins:PluginsVerb",
            "manifests = auto_apms_ros2behavior.verb.node.manifests:ManifestsVerb",
            "call = auto_apms_ros2behavior.verb.node.call:CallVerb",
        ]
    },
)
