from setuptools import find_packages, setup

package_name = 'pros_crane_py'

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
    maintainer='chenfu',
    maintainer_email='kye1234321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "crane = pros_crane_py.crane:main",
            "crane4282_keyboard = pros_crane_py.crane4282_keyboard:main",
            "crane4282_arm_writer = pros_crane_py.crane4282_arm_writer:main",
        ],
    },
)
