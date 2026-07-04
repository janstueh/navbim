from setuptools import setup, find_packages

package_name = 'navbim_bim_server'

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
        'numpy>=2.0,<3',
        'scipy>=1.13.0',
    ],
    zip_safe=True,
    maintainer='Jan Stührenberg',
    maintainer_email='jan.stuehrenberg@tuhh.de',
    description='BIM server for the navbim stack',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'bim_server = navbim_bim_server.bim_server:main',
        ],
    },
)
