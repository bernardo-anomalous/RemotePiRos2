from setuptools import find_packages, setup

package_name = 'remote_pi_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='b',
    maintainer_email='b@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "interface = remote_pi_pkg.interface:main",
            "auv_control = remote_pi_pkg.main:main",
            "gamepad_mapper = remote_pi_pkg.ros.gamepad_mapper:main",
        ],
    },
)
