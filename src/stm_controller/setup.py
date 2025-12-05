from setuptools import setup

package_name = 'stm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juwan',
    maintainer_email='you@example.com',
    description='STM Node: Service server + ESP32 TCP client',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stm_node = stm_controller.stm_node:main'
        ],
    },
)

