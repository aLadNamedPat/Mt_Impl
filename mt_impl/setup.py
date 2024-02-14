from setuptools import find_packages, setup
import os

package_name = 'mt_impl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', package_name, 'launch_impl'), ['launch/launch_impl.py']),
        (os.path.join('share', package_name, 'C-Capt'), ['launch/C-Capt.py']),
        (os.path.join('share', package_name, 'D-Capt'), ['launch/D-Capt.py']),
        (os.path.join('share', package_name, 'D-Capt'), ['launch/D-CaptO.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick',
    maintainer_email='Pattheboy135@gmail.com',
    description="Implementation of Matt Turpin's Chapter 4",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Linear_Int = mt_impl.Linear_Int:main',
            'MultiLinear_Int = mt_impl.MultiLinear_Int:main',
            'ControlNode = mt_impl.ControlNode:main',
            'DCaptNode = mt_impl.DCaptNode:main',
            "TFBroadcaster = mt_impl.TFBroadcaster:main",
            'MT_DCaptNode = mt_impl.MT_DCaptNode:main'
        ],
        'rosidl_cli.command.generate_extensions': [
            'rosidl_generate_py = rosidl_generator_py.cli:GeneratePythonExtension'
        ]
    },
)