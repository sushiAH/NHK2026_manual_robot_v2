from setuptools import find_packages, setup

package_name = 'manual_robot_v2'

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
    maintainer='aratahorie',
    maintainer_email='aratahorie89@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest',],
    },
    entry_points={
        "console_scripts": [
            "subscribe_twist_node = manual_robot_v2.subscribe_twist_node:main",
            "joy2twist_node = manual_robot_v2.joy2twist_node:main",
            "publish_feedback_node = manual_robot_v2.publish_feedback_node:main",
            "control_spear_node = manual_robot_v2.control_spear_node:main",
            "control_box_node = manual_robot_v2.control_box_node:main",
        ],
    },
)
