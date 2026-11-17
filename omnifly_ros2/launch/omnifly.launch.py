import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    share = get_package_share_directory('omnifly')
    cfg_dir = os.path.join(share, 'config')
    with open(os.path.join(cfg_dir, 'drones.yaml')) as f:
        cfg = yaml.safe_load(f)
    robots = {n: r for n, r in cfg['robots'].items() if r.get('enabled')}
    types = cfg['robot_types']
    ctrl = cfg.get('all', {}).get('controller', {})
    warn = cfg.get('all', {}).get('warnings', {}).get('motion_capture', {})
    names = list(robots)
    arg = lambda k: LaunchConfiguration(k).perform(context) == 'true'
    sim = arg('sim')
    nodes = []

    for n, r in robots.items():
        if not sim:
            nodes.append(Node(package='omnifly', executable='crsf_bridge', name=f'{n}_bridge',
                              parameters=[{'drone_name': n, 'serial_port': r['serial_port'],
                                           'dry_run': bool(r.get('dry_run', False))}]))
        nodes.append(Node(package='omnifly', executable='position_controller',
                          name=f'{n}_controller', output='screen',
                          parameters=[{'drone_name': n, **ctrl}]))

    if sim:   # simulator replaces crsf_bridge + mocap
        nodes.append(Node(package='omnifly_sim', executable='sim_server', output='screen'))
    elif arg('mocap'):
        rigid = {}
        for n, r in robots.items():
            mc = types[r['type']]['motion_capture']
            rigid[n] = {'initial_position': r['initial_position'],
                        'marker': mc['marker'], 'dynamics': mc['dynamics']}
        nodes.append(Node(package='motion_capture_tracking',
                          executable='motion_capture_tracking_node',
                          name='motion_capture_tracking', output='screen',
                          parameters=[os.path.join(cfg_dir, 'motion_capture.yaml'),
                                      {'rigid_bodies': rigid}]))
        nodes.append(Node(package='omnifly', executable='mocap_relay',
                          parameters=[{'drone_names': names,
                                       'warning_if_rate_outside':
                                           warn.get('warning_if_rate_outside', [80.0, 120.0])}]))

    if arg('teleop'):
        offsets = []
        for n in names:
            p = robots[n]['initial_position']
            offsets += [p[0], p[1], 0.0]
        nodes.append(Node(package='joy', executable='joy_node', name='joy'))
        nodes.append(Node(package='omnifly', executable='teleop', name='teleop',
                          parameters=[os.path.join(cfg_dir, 'teleop.yaml'),
                                      {'drone_names': names, 'offsets': offsets}]))

    if arg('rviz'):
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                          arguments=['-d', os.path.join(cfg_dir, 'omnifly.rviz')]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('mocap', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
