from setuptools import setup

package_name = 'bcr_teleop'

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
    maintainer='Gaurav Gupta',
    maintainer_email='gaurav@blackcoffeerobotics.com',
    description='Keyboard teleop package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['bcr_teleop_node = bcr_teleop.bcr_teleop_node:main'
                            ],
    },
)
