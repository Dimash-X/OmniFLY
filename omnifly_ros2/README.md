# omnifly (ROS 2)

Pipeline per drone:  /<name>/pose -> position_controller -> /<name>/rc_cmd + /<name>/arm -> crsf_bridge -> ELRS TX -> drone

## Build / run
    cd ~/ws/src && cp -r omnifly . && cd .. && colcon build --packages-select omnifly && source install/setup.bash
    ros2 launch omnifly omnifly.launch.py mocap:=true teleop:=true rviz:=true
    # dry_run: false in config/drones.yaml for real flight
    ros2 service call /drone1/takeoff std_srvs/srv/Trigger
    ros2 run omnifly swarm_sequencer
    ros2 service call /drone1/land std_srvs/srv/Trigger
    ros2 topic pub --once /omnifly/emergency std_msgs/msg/Empty     # EMERGENCY DISARM

## Betaflight setup
- Receiver: CRSF (ELRS), channel order AETR
- Modes: ARM on AUX1 (high), ANGLE on AUX2 (high)
- Calibrate `hover_throttle`; check roll/pitch signs with props OFF first

## Old -> new
| old | new |
|---|---|
| position_controller_1..4.py | position_controller.py (param drone_name) |
| crazyflie_server / cflib | crsf_bridge.py |
| leader_cf, follower_cf | formation_follower.py |
| begin_operation, trajectory, trajectory_publisher | swarm_sequencer.py |
| vel_mux, velocity_controller_test, test_version, mid_operation | removed (superseded) |
| chooser, cfmult, autonomous_sequence | removed (Crazyflie-specific) |
| drone_system_kinematics, drones_object, simple_mapper_multiranger | not ported (cable-load / multiranger, not used by OmniFLY) |

## Config files (old -> new)
| old | new |
|---|---|
| crazyflies.yaml | config/drones.yaml |
| motion_capture.yaml | config/motion_capture.yaml (+ mocap_relay.py) |
| teleop.yaml | config/teleop.yaml (+ teleop.py) |
| server.yaml | merged into drones.yaml `all:` (warnings); sim section dropped |
| config.rviz | config/omnifly.rviz (new minimal) |
| base_config.yaml (Lighthouse) | not used: no Lighthouse deck on this hardware |

## Python API (package omnifly_py, port of crazyflie_py)
    from omnifly_py import OmniFly     # alias: Crazyswarm
    swarm = OmniFly(); allcfs = swarm.allcfs; timeHelper = swarm.timeHelper
    allcfs.takeoff(targetHeight=0.5, duration=3.0); timeHelper.sleep(5.0)
    allcfs.goTo([0.3, 0, 0], 0.0, 3.0); allcfs.land(0.0, 3.0)
See omnifly_py/examples/hello_swarm.py. Needs omnifly launched (controllers + mocap).

## Simulation
    ros2 launch omnifly omnifly.launch.py sim:=true teleop:=true rviz:=true
Runs the real controller against omnifly_sim (no hardware, no mocap). Recommended before every new gain.
