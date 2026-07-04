from setuptools import setup, find_packages

package_name = 'navbim_util'

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
        'shapely',
    ],
    zip_safe=True,
    maintainer='Jan Stührenberg',
    maintainer_email='jan.stuehrenberg@tuhh.de',
    description='Navbim utilities',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_tf_publisher = navbim_util.pose_to_tf_publisher:main',
        ],
    },
)