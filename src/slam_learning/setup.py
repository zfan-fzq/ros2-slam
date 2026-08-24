from setuptools import find_packages, setup

package_name = 'slam_learning'

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
    maintainer='fzq64',
    maintainer_email='fzq64716055@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'slam_publisher = slam_learning.publisher_node:main',
            'slam_subscriber = slam_learning.subscriber_node:main',
            'scan_node = slam_learning.scan_node:main',
            'dynamic_tf = slam_learning.dynamic_tf_node:main',
            'tf_point_listener = slam_learning.tf_point_listener:main',
        ],
    },
)
