from pycrazyswarm import *
import numpy as np

if __name__ == "__main__":

    Z = 1.0

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Takeoff to different heights
    for i, cf in enumerate(allcfs.crazyflies):
        cf.takeoff(targetHeight=0.4 + i * 0.2, duration=4)
    # allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    # timeHelper.sleep(1.5+Z)
    timeHelper.sleep(4)

    # Go to initial positions
    for i, cf in enumerate(allcfs.crazyflies):
        cf.goTo(cf.initialPosition + np.array([0, 0, 0.4 + i * 0.2 ]), 0, 15)
    timeHelper.sleep(15)

    allcfs.land(0.02, 2)