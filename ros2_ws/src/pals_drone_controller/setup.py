from setuptools import setup

setup(
    name="pals_drone_controller", version="0.1.0",
    packages=["pals_drone_controller"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/pals_drone_controller"]),
        ("share/pals_drone_controller", ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="PALS", maintainer_email="pals@example.com",
    description="Local autonomous PX4/MAVROS waypoint controller",
    license="Apache-2.0",
    entry_points={"console_scripts": ["waypoint_demo = pals_drone_controller.node:main"]},
)
