from setuptools import setup
import os
from glob import glob

setup(
    name='omnifly',
    version='0.1.0',
    packages=['omnifly'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/omnifly']),
        ('share/omnifly', ['package.xml']),
        (os.path.join('share', 'omnifly', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'omnifly', 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'crsf_bridge = omnifly.crsf_bridge:main',
        'position_controller = omnifly.position_controller:main',
        'formation_follower = omnifly.formation_follower:main',
        'swarm_sequencer = omnifly.swarm_sequencer:main',
        'mocap_relay = omnifly.mocap_relay:main',
        'teleop = omnifly.teleop:main',
    ]},
)
