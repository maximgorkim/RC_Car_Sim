from setuptools import setup
from glob import glob
import os

package_name = 'traffic_light_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rares Lemnariu',
    maintainer_email='rares.lemnariu@ro.bosch.com',
    description='The traffic_light_pkg package',
    license='Bosch ECC',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tl_talker = traffic_light_pkg.tl_talker:main',
        ],
    },
)
