from setuptools import setup, find_packages

setup(
    name='ustepper_dev',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'rclpy'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'ustepper_dev']),
        ('share/' + 'ustepper_dev', ['package.xml']),
    ],
    zip_safe=True,
    maintainer='yi',
    maintainer_email='0707yi.liu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ustepper_node = ustepper_dev.ustepper_node:main',
        ],
    },
)
