from setuptools import find_packages, setup

package_name = 'oglove_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='He Bin',
    maintainer_email='hebin7611@hotmail.com',
    description='Package for ble and usb glove',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_glove_node = scripts.usb_glove_ros2:main',
            'ble_glove_node = scripts.ble_glove_ros2:main',
        ],
    },
)
