from setuptools import find_packages, setup

package_name = "robcraft_cli"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "ros2cli"],
    zip_safe=True,
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgons@unileon.es",
    description="ROS 2 CLI extensions for RobCraft.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "ros2cli.command": [
            "robcraft = robcraft_cli.command.robcraft:RobcraftCommand",
        ],
        "ros2cli.extension_point": [
            "robcraft = ros2cli.command.CommandExtension",
        ],
    },
)
