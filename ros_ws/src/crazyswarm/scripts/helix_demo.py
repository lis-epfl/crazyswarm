#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import csv
import matplotlib.pyplot as plt


def spiral(t, r, w, x0, y0, z0, phi0):
    x = x0 + r * np.cos(phi0 + w * t)
    y = y0 + r * np.sin(phi0 + w * t)
    z = z0 + 1 - np.cos(t / 5)
    yaw = phi0 + w * t
    pos = np.array([x, y, z])

    return pos, yaw


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.setParam("kalman/resetEstimation", 1)  # Reset kalman filter
    allcfs.setParam("ring/effect", 7)  # Set LED rings to solid color

    cmap = plt.get_cmap('hsv')

    r = 1.5
    w = 2 * np.pi / 10
    z0 = 0.3
    t = 0
    x0 = 2
    y0 = 0
    N = len(allcfs.crazyflies)
    phi0_vec = np.linspace(0, 2 * np.pi, N + 1)
    color_vec = np.array([np.linspace(0, 1, N), np.zeros(N), np.linspace(1, 0, N)]).T

    # Set led color
    for i, cf in enumerate(allcfs.crazyflies):
        # cf.setLEDColor(*color_vec[i])
        color = cmap(float(i) / N)[:3]
        cf.setLEDColor(*color)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    allcfs.takeoff(targetHeight=z0, duration=1.0+z0)
    timeHelper.sleep(1.5+z0)

    # Go to starting position
    for i, cf in enumerate(allcfs.crazyflies):
        pos, yaw = spiral(0.0, r, w, x0, y0, z0, phi0_vec[i])
        cf.goTo(pos, yaw, 4)
    timeHelper.sleep(4)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    # Fly spiral
    while t < 20:
        for i, cf in enumerate(allcfs.crazyflies):
            pos, yaw = spiral(t, r, w, x0, y0, z0, phi0_vec[i])
            cf.cmdPosition(pos, yaw * 180 / np.pi)

        t += 0.05
        timeHelper.sleep(0.05)

    # Land
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop(remainValidMillisecs=100)
        # cf.goTo(cf.initialPosition + np.array([0, 0, z0]), 0, 7)
        cf.goTo(cf.initialPosition + np.array([0, 0, z0]), 0, 5)
    timeHelper.sleep(5)

    allcfs.land(targetHeight=0.02, duration=2.0+z0)
    timeHelper.sleep(1.0+z0)
