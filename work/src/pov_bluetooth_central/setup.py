from setuptools import setup, find_packages

package_name = 'pov_bluetooth_central'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts',
            ['scripts/bluetooth_central_node']),
    ],
    install_requires=['setuptools', 'bleak>=0.22.3'],
    zip_safe=True,
    maintainer='ShinjiEto',
    maintainer_email='shinji1095nameko@gmail.com',
    description='ROS 2 package for Bluetooth central communication with XIAO nRF52840',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluetooth_central_node = pov_bluetooth_central.bluetooth_central_node:main',
        ],
    },
)