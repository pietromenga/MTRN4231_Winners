from setuptools import setup

package_name = 'winners_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'dual_camera_test',  # Change this to the name of your Python node without the .py extension
    ],
    install_requires=['setuptools'],
    data_files=[],
    entry_points={
        'console_scripts': [
            'dual_camera_test = dual_camera_test:main',  # Change accordingly
        ],
    },
)
