from setuptools import find_packages, setup

package_name = 'blueboat_control'

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
    maintainer='blueboat_sitl',
    maintainer_email='blueboat_sitl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asv_pid_rc = blueboat_control.asv_pid_rc_node:main',
        ],
    },
)
