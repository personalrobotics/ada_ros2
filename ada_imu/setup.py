from setuptools import setup

package_name = 'ada_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayabean',
    maintainer_email='hayabean@gmail.com',
    description='IMU jointstate publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_jointstate_publisher = ada_imu.imu_jointstate_publisher:main'
        ],
    },
)
