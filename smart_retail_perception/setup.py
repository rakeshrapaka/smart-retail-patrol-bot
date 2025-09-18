from setuptools import setup

package_name = 'smart_retail_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rakesh',
    maintainer_email='you@example.com',
    description='Human detection node using OpenCV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_node = smart_retail_perception.perception_node:main',
        ],
    },
)
