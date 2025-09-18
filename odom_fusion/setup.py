from setuptools import setup

package_name = 'odom_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rakesh',
    maintainer_email='you@example.com',
    description='Odometry fusion node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
