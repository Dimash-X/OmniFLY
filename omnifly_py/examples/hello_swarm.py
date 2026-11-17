"""Minimal example: take off, move the whole swarm, land."""
import numpy as np
from omnifly_py import OmniFly

TAKEOFF_HEIGHT = 0.5


def main():
    swarm = OmniFly()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=3.0)
    timeHelper.sleep(5.0)

    allcfs.goTo(np.array([0.3, 0.0, 0.0]), 0.0, 3.0)   # relative offset
    timeHelper.sleep(4.0)

    for cf in allcfs.crazyflies:                        # individual command
        cf.goTo(np.array([0.0, 0.0, 0.1]), 0.0, 2.0, relative=True)
    timeHelper.sleep(3.0)

    allcfs.land(targetHeight=0.0, duration=3.0)
    timeHelper.sleep(5.0)


if __name__ == '__main__':
    main()
