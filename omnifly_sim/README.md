# omnifly_sim

Software simulation of OmniFLY drones. Replaces `crsf_bridge` + mocap, so the REAL
`position_controller`, `teleop`, `swarm_sequencer` and `omnifly_py` scripts run unchanged:

    ros2 launch omnifly omnifly.launch.py sim:=true teleop:=true rviz:=true

Backends: `np` (angle-mode physics) or `none` (drone jumps to /<name>/goal).
Visualizations: rviz, pdf (one file per drone), record_states, blender.
Blender needs `bpy` and a drone model + env image under visualization/data/
(model/<name>.obj, env/env.jpg); you can reuse the ones from crazyflie_sim or supply your own.
Physics parameters in config/sim.yaml are APPROXIMATE: measure mass and thrust of your drone.
