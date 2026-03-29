import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mina_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install all ONNX models inside the policies directory
        (os.path.join('share', package_name, 'policies'), glob('policies/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattia',
    maintainer_email='mattia.prandi@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            "dynamic_switch = mina_control.dynamic_switch:main",
            "mina_fsm = mina_control.mina_fsm:main",
            "mina_vmc = mina_control.mina_vmc:main",
            "rl_controller = mina_control.rl_controller:main",
            'switch_test_adapter = mina_control.switch_test_adapter:main',
        ],
    },
)
