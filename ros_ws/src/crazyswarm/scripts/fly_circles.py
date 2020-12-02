#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import csv
import time

Z = 0.5 # Takeoff height

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.setParam("stabilizer/controller", 2)  # Set controller
    
    # Reset kalman filer by setting kalman/resetEstimation to 1 and back to 0
    allcfs.setParam("kalman/resetEstimation", 1)  # Set controller
    allcfs.setParam("kalman/resetEstimation", 0)  # Set controller

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    mode = 'cmdFullState'  # 'cmdFullState' or 'cmdPosition'

    cmd_pos_list = []
    meas_pos_list = []
    time_list = []
    r = 0.75
    w = 2 * np.pi / 8
    t = 0
    dt = 0.1

    for cf in allcfs.crazyflies:
        pos = np.array([r * np.cos(w * t), r * np.sin(w * t), Z])
        cf.goTo(cf.initialPosition + pos, 0, 2)
    timeHelper.sleep(2)


    t0 = time.time()

    while t < 8:
        pos = np.array([r * np.cos(w * t), r * np.sin(w * t), Z])
        vel = np.array([-r * w * np.sin(w * t), r * w * np.cos(w * t), 0])
        acc = np.array([-r * w * w * np.cos(w * t), -r * w * w * np.sin(w * t), 0])
        yaw = w * t
        omega = np.array([0, 0, w])

        time_list.append(timeHelper.time())
        cmd_pos_list.append(cf.initialPosition + pos)
        meas_pos_list.append(cf.position())


        for cf in allcfs.crazyflies:
            if mode == 'cmdFullState':
                cf.cmdFullState(cf.initialPosition + pos, vel, acc, yaw, omega)
            elif mode == 'cmdPosition':
                cf.cmdPosition(cf.initialPosition + pos, yaw * 180 / np.pi)
            else:
                Exception('Unknown mode')

        t += dt
        timeHelper.sleepForRate(1 / dt)

    t1 = time.time()
    print(t1- t0)

    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop(remainValidMillisecs=100)
        cf.goTo(cf.initialPosition + np.array([0, 0, Z]), 0, 2)

    timeHelper.sleep(2)

    allcfs.land(targetHeight=0.02, duration=2.0+Z)
    timeHelper.sleep(1.0+Z)

    file_name = 'test_' + mode + '.csv'
    with open(file_name, 'w') as file:
        writer = csv.writer(file)
        cmd_pos_array = np.array(cmd_pos_list)
        meas_pos_array = np.array(meas_pos_list)
        writer.writerow(time_list)
        writer.writerow(cmd_pos_array[:, 0])
        writer.writerow(cmd_pos_array[:, 1])
        writer.writerow(cmd_pos_array[:, 2])
        writer.writerow(meas_pos_array[:, 0])
        writer.writerow(meas_pos_array[:, 1])
        writer.writerow(meas_pos_array[:, 2])