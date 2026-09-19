from setuptools import find_packages, setup

package_name = 'Week4_lab'

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
    maintainer='ben',
    maintainer_email='famousgrouse@live.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'closest_scan_marker_node = Week4_lab.closest_scan_marker_ANSWER:main',
            'safe_stop_node = Week4_lab.safe_stop_SOLUTION:main',
            'reactive_avoid_node = Week4_lab.reactive_avoid_SOLUTION:main', # for the reactive avoidance task

        ],
    },
)
