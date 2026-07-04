from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'navbim_nav_model_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'dash',
        'matplotlib>=3.8.0',
        'numpy>=2.0,<3',
        'open3d>=0.18.0',
        'plotly',
        'shapely>=2.0.0',
        'networkx',
        'psutil',
        'ladybug_geometry_polyskel',
    ],
    zip_safe=True,
    maintainer='Jan Stührenberg',
    maintainer_email='jan.stuehrenberg@tuhh.de',
    description='Generation of the navigation model (topological map and occupancy maps) from BIM models for the navbim stack',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'nav_model_generator_node = navbim_nav_model_gen.generator_manager:main',
        ],
    },
)