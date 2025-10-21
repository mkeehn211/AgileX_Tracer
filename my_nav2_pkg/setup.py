from setuptools import setup
import os
from glob import glob

package_name = 'my_nav2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install all config files (yaml, pgm, etc.)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael',
    maintainer_email='mkeehn211@gmail.com',
    description='Custom Nav2 bringup for my robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)

