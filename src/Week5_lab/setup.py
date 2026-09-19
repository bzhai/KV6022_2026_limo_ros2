from setuptools import find_packages, setup

package_name = 'Week5_lab'

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
            'limo_vision_bridge_node = Week5_lab.limo_vision_SOLUTION:main',
            'colour_contours_node = Week5_lab.colour_contours_SOLUTION:main',
        ],
    },
)
