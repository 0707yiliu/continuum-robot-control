from setuptools import find_packages, setup

package_name = 'rob_central'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rob_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/geomagic_2dynamixel_1ustepper_control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yi',
    maintainer_email='0707yi.liu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'central_node = rob_central.central_node:main'
        ],
    },
)
