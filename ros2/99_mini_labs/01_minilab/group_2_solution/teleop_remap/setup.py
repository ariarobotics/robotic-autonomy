from setuptools import find_packages, setup

package_name = 'teleop_remap'

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
    maintainer='AdamBass',
    maintainer_email='adambass2036@gmail.com',
    description='Remaps a Twist message from one topic to another, reversing its direction in the process',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker_listener = teleop_remap.teleop_remap:main',
        ],
    },
)
