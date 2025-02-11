from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'GUI'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjj73',
    maintainer_email='kjjbusiness73@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'App_v1 = GUI.UserApp.App_v1:main',
            'Manager_v1 = GUI.Admin.Manager_v1:main',
            'Kiosk_v1 = GUI.Kiosk.iosk_v1:main',
        ],
    },
)
