#!/usr/bin/env python3
"""System tests for OpenDrone-V2: launches the drone stack, flies every test of opendrone_config.TESTS,
records the position topic in a rosbag, converts it to csv and plots it against the ideal trajectory in a pdf."""
from subprocess import Popen, PIPE, TimeoutExpired
import time
import os
import signal
from datetime import datetime
from pathlib import Path
import shutil
import atexit

import opendrone_config as cfg
from mcap_handler import McapHandler
from plotter_class import Plotter


def clean_process(process:Popen) -> int :
    '''Kills process and its children on exit if they aren't already terminated (called with atexit). Returns 0 on success, 1 on failure'''

    if process.poll() is None:
        group_id = os.getpgid(process.pid)
        print(f"cleaning process {group_id}")
        os.killpg(group_id, signal.SIGTERM)
        time.sleep(0.01) #necessary delay before first poll
        i = 0
        while i < 10 and process.poll() is None:  #in case the termination is lazy we wait up to 0.5 s
            time.sleep(0.05)
            i += 1
        if process.poll() is None:
            print(f"Process group {process} with groupID {group_id} didn't terminate correctly")
            return 1
        return 0
    return 0 #process already terminated


def record_start_and_clean(testname:str, max_wait:int, bagfolder:Path, setup_script:Path):
    '''Starts recording cfg.RECORDED_TOPIC in a rosbag, starts the test, waits, closes the rosbag and terminates all processes.
        max_wait is the max amount of time to wait before forcefully terminating the test flight script (in case it never finishes correctly).
        NB the testname must be the name of an executable of cfg.ROS_RUN_PACKAGE (ie "ros2 run <ROS_RUN_PACKAGE> testname" must be valid)'''

    src = f"source {setup_script}"
    bagname = bagfolder.name   # "bag_<date>"

    record_bag = None
    start_flight_test = None
    try:
        command = f"{src} && ros2 bag record -s mcap -o {testname}_{bagname} {cfg.RECORDED_TOPIC}"
        record_bag = Popen(command, shell=True, cwd=bagfolder, stderr=PIPE, stdout=True,
                           start_new_session=True, text=True, executable="/bin/bash")
        atexit.register(clean_process, record_bag)

        command = f"{src} && ros2 run {cfg.ROS_RUN_PACKAGE} {testname}"
        start_flight_test = Popen(command, shell=True, stderr=True, stdout=True,
                                  start_new_session=True, text=True, executable="/bin/bash")
        atexit.register(clean_process, start_flight_test)

        start_flight_test.wait(timeout=max_wait)  #raises TimeoutExpired after max_wait seconds if the test didn't finish by itself
    except (TimeoutExpired, KeyboardInterrupt):   #max_wait exceeded, or the drone crashed and the user pressed ^C to skip the waiting
        pass
    finally:
        if start_flight_test is not None:
            clean_process(start_flight_test)
        if record_bag is not None:
            clean_process(record_bag)

    #if something went wrong with the bash command lines in Popen, print the error
    if record_bag is not None and record_bag.stderr is not None:
        print(testname, " record_bag stderr: ", record_bag.stderr.readlines())
    if start_flight_test is not None and start_flight_test.stderr is not None:
        print(testname, " start_flight flight stderr: ", start_flight_test.stderr.readlines())


def translate_and_plot(testname:str, bagfolder:Path, results_dir:Path, now:str, systemtests_dir:Path):
    '''Translates rosbag .mcap format to .csv, then uses that csv to plot a pdf '''

    bagname = bagfolder.name
    # NB : the mcap filename is almost the same as the folder name but has _0 at the end
    inputbag = str(bagfolder / f"{testname}_{bagname}" / f"{testname}_{bagname}_0.mcap")
    output_csv = str(bagfolder / f"{testname}_{bagname}" / f"{testname}_{bagname}_0.csv")
    McapHandler().write_mcap_to_csv(inputbag, output_csv)  #translate bag from mcap to csv

    output_pdf = str(results_dir / f"Results_{cfg.PROJECT_NAME}_{testname}_{now}.pdf")
    test_file = cfg.TESTS[testname]["reference_csv"]       #relative to plotter_class.py
    Plotter().create_figures(test_file, output_csv, output_pdf, testname)  #plot the data


if __name__ == "__main__":

    path = Path(__file__)  #this file sits in <workspace>/src/<repo>/systemtests/, so path.parents[3] is the ROS 2 workspace

    workspace = path.parents[3]
    bags_root = workspace / "bagfiles"
    results_dir = workspace / "results"
    setup_script = workspace / "install" / "setup.bash"

    #delete results, logs and bags of previous experiments if they exist
    if bags_root.exists():
        shutil.rmtree(bags_root)
    if results_dir.exists():
        shutil.rmtree(results_dir)
    if (Path.home() / ".ros/log").exists():
        shutil.rmtree(Path.home() / ".ros/log")

    #create the folder where we will record the different bags and the folder where the results pdf will be saved
    now = datetime.now().strftime('%d_%m_%Y-%H_%M_%S')
    bagfolder = bags_root / f"bag_{now}"
    os.makedirs(bagfolder)
    os.makedirs(results_dir)

    command = f"source {setup_script} && {cfg.ROS_LAUNCH_CMD}"
    launch_drone = Popen(command, shell=True, stderr=True, stdout=True, text=True,
                         start_new_session=True, executable="/bin/bash")
    atexit.register(clean_process, launch_drone)  #atexit makes sure processes are cleaned even if the script exits unexpectedly

    time.sleep(cfg.LAUNCH_STARTUP_WAIT_S)
    for testname, test in cfg.TESTS.items():
        record_start_and_clean(testname, test["timeout_s"], bagfolder, setup_script)

    clean_process(launch_drone)   #kill the drone stack and all of its child processes

    #tests done, now we create the results pdfs
    for testname in cfg.TESTS:
        translate_and_plot(testname, bagfolder, results_dir, now, path.parent)

    exit(0)
