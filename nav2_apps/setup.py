import os
from setuptools import setup
from glob import glob

package_name = 'nav2_apps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='morg1207',
    maintainer_email='alaurao@uni.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'approach_srv = nav2_apps.approach_server:main',
            'patrol_behavior = nav2_apps.server_patrol_behavior:main',
            'shelf_position_srv = nav2_apps.server_shelf_position:main',
            'auto_localization = nav2_apps.auto_localization:main',
            'cart_frame = nav2_apps.cart_frame:main',
            'main_node = nav2_apps.main_server:main'
        ],
    },
)
