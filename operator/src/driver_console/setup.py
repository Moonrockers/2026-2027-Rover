from setuptools import find_packages, setup

package_name = "driver_console"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bryson",
    maintainer_email="15smith.bryson@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "driver_console = driver_console.driver_console:main",
            "autonomy_controller = driver_console.autonomy_controller:main",
        ],
    },
)
