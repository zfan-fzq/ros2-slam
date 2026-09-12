from setuptools import find_packages, setup

package_name = 'astar_learning'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='fzq64',
    maintainer_email='fzq64@example.com',

    description='A* global planner learning package',
    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'planner_node = astar_learning.planner_node:main',
        ],
    },
)
