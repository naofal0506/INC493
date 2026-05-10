from setuptools import setup
import os
from glob import glob

package_name = 'my_f1tenth_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    description='F1Tenth All-in-One Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'newodom_node = my_f1tenth_pkg.newodom:main',
            'odom_erpm_node = my_f1tenth_pkg.odom_erpm:main',
            'odom_tacho_node = my_f1tenth_pkg.odom_tacho:main',
        ],
    },
)
