from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'm20'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # ⭐ 安装 launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xxx',
    maintainer_email='xxx',
    description='m20 odom + tf package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # ⭐ 注册节点
            'simple_odom = m20.simple_odom:main',
            'simple_odom_old = m20.simple_odom_old:main',
        ],
    },
)