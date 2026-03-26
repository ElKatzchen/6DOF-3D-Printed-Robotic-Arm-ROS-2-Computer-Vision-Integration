from setuptools import find_packages, setup

package_name = 'robotic_arm_control'

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
            'pubsub = robotic_arm_control.pubsub:main',
            'sub = robotic_arm_control.sub:main',
        ],
    },
)
