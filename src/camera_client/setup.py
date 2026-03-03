import os
from glob import glob

from setuptools import find_packages, setup

package_name = "camera_client"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools", "requests"],
    zip_safe=True,
    maintainer="arnav",
    maintainer_email="arnav@todo.todo",
    description="Camera client node that forwards images to a FastAPI server based on distance traveled",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_client_node = camera_client.camera_client_node:main",
        ],
    },
)
