#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Parameters
    Z = 0.5  # Takeoff height
    w = 2 * np.pi / 8  # Angular velocity
    t = 0
    dt = 0.1  # Time discretization

    # NOTE: cmdHover ONLY works with PID so far!
    allcfs.setParam("stabilizer/controller", 1)  # Set controller to PID
    # Reset kalman filer by setting kalman/resetEstimation to 1 and back to 0
    allcfs.setParam("kalman/resetEstimation", 1)

    # Takeoff
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    # Fly in circles
    while t < 8:
        for cf in allcfs.crazyflies:
            cf.cmdHover(0.8, 0.0, -w * 180 / np.pi, Z)
        t += dt
        timeHelper.sleepForRate(1 / dt)

    # Move crazyflies back to initial position
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop(remainValidMillisecs=100)
        cf.goTo(cf.initialPosition + np.array([0, 0, Z]), 0, 4)
    timeHelper.sleep(4)

    # Land
    allcfs.land(targetHeight=0.02, duration=2.0+Z)
    timeHelper.sleep(1.0+Z)
