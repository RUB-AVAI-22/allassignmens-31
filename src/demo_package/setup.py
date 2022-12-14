from setuptools import setup

package_name = 'demo_package'


setup(
    name=package_name,
    version='2022.11.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas Kuhl',
    maintainer_email='nicolas.kuhl@rub.de',
    description='This package contains the default demo publisher and subscriber',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = demo_package.pub:main',
            'sub = demo_package.sub:main',
            'User_Node = demo_package.User_Node:main',
            'control_turtle = demo_package.control_turtle:main',
            'Camera_Node = demo_package.Camera_Node:main',
            'Ai_Node = demo_package.Ai_Node:main',
            'TestLidar = demo_package.TestLidar:main',

        ],
    },
)
