from setuptools import find_packages, setup
from setuptools import setup
from glob import glob
import os

package_name = 'gazebo_modele'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'world'), glob('world/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boxing/喵了个水蓝蓝',
    maintainer_email='clibang2022@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps = gazebo_modele.gps:main',
            'qt = gazebo_modele.qt:main',
        ],
    },
)
