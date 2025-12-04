from setuptools import setup, find_packages

package_name = 'dqn_robot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Necesario para que ROS2 detecte el paquete
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Instalar package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='camila',
    maintainer_email='camila.monje@ucb.edu.bo',
    description='Deep Q-Network TurtleBot Navigation',
    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            # Ejecutables ROS2
            'train_node = dqn_robot_nav.train_node:main',
            'test_node = dqn_robot_nav.test_node:main',
        ],
    },
)
