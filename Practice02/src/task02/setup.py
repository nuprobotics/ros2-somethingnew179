from setuptools import setup

package_name = 'task02'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=['publisher'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/task02.yaml']),
        ('share/' + package_name + '/launch', ['launch/task02.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer.name',
    maintainer_email='maintainer@email.com',
    description='ROS2 practice task 02',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = publisher:main',
        ],
    },
)
