from setuptools import setup

package_name = 'week4_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display_arm.launch.py']),  
        ('share/' + package_name + '/urdf', ['urdf/2link_robot.urdf']),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ankit Pandey',
    maintainer_email='ankit@example.com',
    description='Python-based ROS 2 package for Week 4A',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kinematics_node = week4_arm.forward_kinematics_node:main',
            'inverse_kinematics_node = week4_arm.inverse_kinematics_node:main',
            'forward_kinematics_3d_node = week4_arm.forward_kinematics_3d_node:main',

        ],
    },
)