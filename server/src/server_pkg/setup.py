from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'server_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'server_pkg': ['waypoints.yaml', 'dashboard.html','static/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask', 'pyyaml'],
    zip_safe=True,
    maintainer='asd',
    maintainer_email='asd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'modi_bridge = server_pkg.modi_bridge:main'
        ],
    },
)
