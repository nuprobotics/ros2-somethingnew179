from setuptools import setup

package_name = 'task03'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=['trigger_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/task03.yaml']),
        ('share/' + package_name + '/launch', ['launch/task03.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer.name',
    maintainer_email='maintainer@email.com',
    description='ROS2 practice task 03',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_node = trigger_node:main',
        ],
    },
)
