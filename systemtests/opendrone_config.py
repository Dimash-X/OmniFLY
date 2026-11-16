"""Project settings for the OpenDrone-V2 system tests.

Edit only this file to adapt run_test.py and plotter_class.py to the drone and the flight program.
Every line marked CHANGE is a placeholder that has to match the real project.
"""

PROJECT_NAME = "OpenDrone-V2"

# Printed on the title page of every results PDF
DRONE = {
    "frame": '3" carbon frame, 141 mm wheelbase',
    "motors": "CHANGE: motor model",            # e.g. the exact 9x9 / 12x12 mount motor you fly
    "propellers": '3" 3-blade',                 # CHANGE if the prop model should be named
}

# --- ROS 2 side (CHANGE all of these to the real package and launch names) ---
ROS_LAUNCH_CMD = "ros2 launch opendrone launch.py"   # starts the drone driver / simulator
ROS_RUN_PACKAGE = "opendrone_examples"               # package whose executables are the flight tests
LAUNCH_STARTUP_WAIT_S = 1                            # wait after the launch before the first test
RECORDED_TOPIC = "/tf"                               # mcap_handler.py expects tf2_msgs/TFMessage here

# --- Pass / fail ---
EPSILON_M = 0.05          # max allowed distance between ideal and recorded position [m]
SAMPLE_PERIOD_S = 0.01    # used only to convert "number of bad samples" into seconds

# --- Tests, run in this order. The key is the name of the executable in ROS_RUN_PACKAGE ---
TESTS = {
    "figure8": {
        "title": "Figure8",
        "timeout_s": 20,
        "reference_csv": "../opendrone_examples/opendrone_examples/data/figure8.csv",
        "delay_s": 4.75,                   # time between the start of the recording and the start of the trajectory
        "altitude_m": 1.0,                 # takeoff altitude set in the test program
        "altitude_from_trajectory": False, # False: z stays at altitude_m, True: altitude_m + z of the trajectory
    },
    "multi_trajectory": {
        "title": "Multi_trajectory",
        "timeout_s": 80,
        "reference_csv": "../opendrone_examples/opendrone_examples/data/multi_trajectory/traj0.csv",
        "delay_s": 4.75,
        "altitude_m": 1.0,
        "altitude_from_trajectory": True,
    },
}
