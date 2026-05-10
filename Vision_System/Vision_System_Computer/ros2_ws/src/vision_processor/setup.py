from setuptools import find_packages, setup
import os

package_name = 'vision_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), [
            'vision_processor/calib_upside_data.npz', 
            'vision_processor/calib_side_data.npz'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='katzchen',
    maintainer_email='sesecastro.s@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pubsub = vision_processor.pubsub:main',
        ],
    },
)
