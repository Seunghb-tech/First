from setuptools import setup

package_name = 'hocarmini_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hocar_subscriber_node = hocarmini_control_pkg.hocar_subscriber_node:main',
            'hocar_control_node = hocarmini_control_pkg.hocar_control_node:main',
            'lidar_subscriber = hocarmini_control_pkg.lidar_subscriber_node:main'
        ],
    },
)
