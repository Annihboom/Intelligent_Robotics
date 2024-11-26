from setuptools import find_packages, setup
from glob import glob

package_name = 'my_mcl_pkg'

setup(
    name=package_name,
    version='0.0.1',  # 确保版本号正确
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')), 
        ('share/' + package_name + '/map', glob('map/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'transforms3d',
        'rclpy',         # ROS 2 Python客户端库
        'sensor-msgs',   # 支持 LaserScan 消息
        'std-msgs',      # 支持 Bool 消息
        'geometry-msgs', # 支持 Twist 消息
    ],
    zip_safe=True,
    maintainer='chjm',
    maintainer_email='q2523062390@126.com',
    description='A Monte Carlo Localization package for ROS 2.',
    license='BSD',  # 修改为实际许可证
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_mcl = my_mcl_pkg.test_mcl:main',
            'gt_tf = my_mcl_pkg.gt_tf_publisher:main',
            'test_map = my_mcl_pkg.test_map:main',
            'test_scan = my_mcl_pkg.test_scan:main',
            'test_odom = my_mcl_pkg.test_odom:main',
            'safe_stop = my_mcl_pkg.safe_stop:main',
            'safe_stop_cmd= my_mcl_pkg.safe_stop_cmd:main',
        ],
    },
)
