from setuptools import setup

package_name = 'simple_lidar'

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
    maintainer='khouly',
    maintainer_email='khouly@todo.todo',
    description='Simple LDS-01 lidar decoder and LaserScan publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reader = simple_lidar.reader:main',
            'capture_raw = simple_lidar.capture_raw:main',
	    'probe = simple_lidar.probe:main',
        ],
    },
)
