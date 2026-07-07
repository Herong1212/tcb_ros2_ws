from setuptools import find_packages, setup
from glob import glob
import os

package_name = "tl_robot_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "meshes/TCB605_05N"),
            glob("meshes/TCB605_05N/*.STL"),
        ),
        (
            os.path.join("share", package_name, "meshes/TCB610_06N"),
            glob("meshes/TCB610_06N/*.STL"),
        ),
        (
            os.path.join("share", package_name, "meshes/TCB705_05N"),
            glob("meshes/TCB705_05N/*.STL"),
        ),
        (
            os.path.join("share", package_name, "meshes/TCB710_06N"),
            glob("meshes/TCB710_06N/*.STL"),
        ),
        (
            os.path.join("share", package_name, "meshes/xwd_urdf_new"),
            glob("meshes/xwd_urdf_new/*.STL"),
        ),
        (
            os.path.join("share", package_name, "meshes/composite_robot"),
            glob("meshes/composite_robot/*.STL"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duyukun",
    maintainer_email="duyukun@fdrobot.com",
    description="TCB robot description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
