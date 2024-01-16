from setuptools import setup

package_name = 'ros2_amr'

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
    maintainer='Harsh Bhatt',
    maintainer_email='hubhatt09@example.com',
    description='Example package for sending navigation goals',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_nav_goal = ros2_amr.go_to_goal:main',
        ],
    },
)

