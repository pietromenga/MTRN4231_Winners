from setuptools import setup, find_packages

package_name = 'util_arduino_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='Simple arduino serial communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'util_arduino_serial = util_arduino_serial.util_arduino_serial:main'
        ],
    },
)
