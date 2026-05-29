from setuptools import setup

package_name = 'aeb_fuzzy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albin',
    maintainer_email='albin@example.com',
    description='Fuzzy Logic AEB node for CARLA',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aeb_fuzzy_node = aeb_fuzzy.aeb_fuzzy_node:main',
        ],
    },
)
