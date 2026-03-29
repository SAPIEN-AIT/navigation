from setuptools import find_packages, setup

package_name = 'mina_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattia',
    maintainer_email='mattia.prandi@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            "can_driver = mina_hardware.CAN_driver:main",
            "imu_driver = mina_hardware.IMU_driver:main",
        ],
    },
)
