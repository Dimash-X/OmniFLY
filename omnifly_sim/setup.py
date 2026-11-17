from glob import glob
from setuptools import find_packages, setup

package_name = 'omnifly_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OmniFLY',
    maintainer_email='you@example.com',
    description='Simulation of OmniFLY drones',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'sim_server = omnifly_sim.sim_server:main',
    ]},
)
