from setuptools import setup

package_name = 'task01'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=['receiver'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task01.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer.name',
    maintainer_email='maintainer@email.com',
    description='TODO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver = receiver:main',
        ],
    },
)
