from setuptools import setup, find_packages

package_name = 'pycarmaker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='ROS 2 package for CarMaker APO telemetry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carmakercamera = pycarmaker.carmakercamera:main',
        ],
    },
)
