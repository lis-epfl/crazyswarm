#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Parameters
    Z = 0.5  # Takeoff height
    r = 0.75  # Circle radius
    w = 2 * np.pi / 8  # Angular velocity
    t = 0
    dt = 0.1  # Time discretization

    mode = 'cmdFullState'  # 'cmdFullState' or 'cmdPosition'

    allcfs.setParam("stabilizer/controller", 2)  # Set controller
    
    # Reset kalman filer by setting kalman/resetEstimation to 1 and back to 0
    allcfs.setParam("kalman/resetEstimation", 1)
    allcfs.setParam("kalman/resetEstimation", 0)

    # Takeoff
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    # Move crazyflies to position on circle
    for cf in allcfs.crazyflies:
        pos = np.array([r * np.cos(w * t), r * np.sin(w * t), Z])
        cf.goTo(cf.initialPosition + pos, 0, 2)
    timeHelper.sleep(2)

    # Fly in circles
    while t < 8:
        pos = np.array([r * np.cos(w * t), r * np.sin(w * t), Z])
        vel = np.array([-r * w * np.sin(w * t), r * w * np.cos(w * t), 0])
        acc = np.array([-r * w * w * np.cos(w * t), -r * w * w * np.sin(w * t), 0])
        yaw = w * t
        omega = np.array([0, 0, w])

        for cf in allcfs.crazyflies:
            if mode == 'cmdFullState':
                cf.cmdFullState(cf.initialPosition + pos, vel, acc, yaw, omega)
            elif mode == 'cmdPosition':
                cf.cmdPosition(cf.initialPosition + pos, yaw * 180 / np.pi)
            else:
                Exception('Unknown mode')

        t += dt
        timeHelper.sleepForRate(1 / dt)

    # Move crazyflies back to initial position
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop(remainValidMillisecs=100)
        cf.goTo(cf.initialPosition + np.array([0, 0, Z]), 0, 2)
    timeHelper.sleep(2)

    # Land
    allcfs.land(targetHeight=0.02, duration=2.0+Z)
    timeHelper.sleep(1.0+Z)
