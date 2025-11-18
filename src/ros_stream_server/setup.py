import glob
import os
from setuptools import find_packages, setup

package_name = "ros_stream_server"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob.glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools", "pyzmq", "ultralytics", "cv_bridge"],
    zip_safe=True,
    maintainer="seoyc",
    maintainer_email="seo.youngchae@lgepartner.com",
    description="This is a ROS2 package for streaming camera image data",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["stream_server = ros_stream_server.stream_server:main"],
    },
)
