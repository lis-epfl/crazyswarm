#!/usr/bin/env python

import rospy
import numpy as np
from pycrazyswarm import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece, FullState, Position, VelocityWorld

Z = 0.5

def reynolds(positions, velocities=None, separation_gain=1.0, cohesion_gain=1.0,
             alignment_gain=1.0, perception_radius=None, max_agents=None):
    """Reynolds flocking for a single agent.

    Args:
        positions: List of relative positions to other agents.
        velocities: List of velocities of other agents (optional).
        separation_gain: Scalar gain for separation component.
        cohesion_gain: Scalar gain for cohesion component.
        alignment_gain: Scalar gain for alignment component.
        perception_radius: Scalar metric distance.
        max_agents: Maximum number of agents to include.

    Returns:
        command: Velocity command.

    """

    positions = np.array(positions)
    if velocities is None:
        velocities = np.zeros_like(positions)
    else:
        velocities = np.array(velocities)
    num_agents, dims = positions.shape

    indices = np.arange(num_agents)
    # Filter agents by metric distance
    distances = np.linalg.norm(positions, axis=1)
    if perception_radius is not None:
        indices = distances < perception_radius
        distances = distances[indices]

    # Filter agents by topological distance
    if max_agents is not None:
        indices = distances.argsort()[:max_agents]
        distances = distances[indices]

    # Return early if no agents
    if len(distances) == 0:
        return np.zeros(dims)

    # Compute Reynolds flocking only if there is an agent in range
    positions = positions[indices]
    velocities = velocities[indices]
    dist_inv = positions / distances[:, np.newaxis] ** 2

    # Renolds command computations
    separation = -separation_gain * dist_inv.mean(axis=0)
    cohesion = cohesion_gain * positions.mean(axis=0)
    alignment = alignment_gain * velocities.mean(axis=0)

    return separation + cohesion + alignment

def get_positions(allcfs):
    n_cfs = len(allcfs.crazyflies)
    positions = [np.zeros(3) for _ in range(n_cfs)]
    for i, cf in enumerate(allcfs.crazyflies):
        positions[i] = cf.position()

    return positions

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs


    timeHelper.sleep(5.0)

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    # Waypoint
    waypoint = np.array([3.0, 3.0, 1.0])
    waypoint = None
    params = {
        'separation_gain': 1.0,         # [unitless]
        'cohesion_gain': 0.5,           # [unitless]
        'alignment_gain': 0.0,          # [unitless]
        'migration_gain': 1.0,          # [unitless]
        'pref_velocity': 1.0,           # prefered flocking velocity [m/s]
        'max_velocity': 2.0,            # veloctiy capped after [m/s]
        'side_length': 4.0,             # position sampling in cube [m]
        'min_distance': 1.5,            # min distance in cube [m]
        'perception_radius': 5.0,      # radius of perception [m]
        'max_agents': 100,              # influenced by max num of agents
        'acceptance_radius': 1.0,       # waypoint acceptance [m]
        'low_pass_gain': 0.9,           # gain for lowpass filter
    }

    t = 0
    while t < 10:

        positions = np.array(get_positions(allcfs), dtype=np.float)

        if waypoint is not None:

            # Check if any agent is close to the waypoint
            min_waypoint_dist = np.linalg.norm(
                positions - waypoint, axis=1).min()

            # Waypoint navigation
            if min_waypoint_dist < params['acceptance_radius']:
                rospy.loginfo('Waypoint reached: {}'.format(waypoint))
                waypoint = None

        # Main flocking loop
        for i, cf in enumerate(allcfs.crazyflies):

            # Final velocity command for agent i
            velocity = np.zeros(3)
            velocity[0] = np.sin(2 * np.pi / 3 * t)
            velocity[1] = np.cos(2 * np.pi / 3 * t)

            my_pos = positions[i]
            other_pos = np.vstack((positions[:i], positions[i + 1:]))
            rel_pos = other_pos - my_pos

            # Add migration command
            if waypoint is not None:
                rel_waypoint = waypoint - my_pos
                migration = rel_waypoint / np.linalg.norm(rel_waypoint)
                velocity += params['migration_gain'] * migration

            command = reynolds(rel_pos, velocities=None,
                               separation_gain=params['separation_gain'],
                               cohesion_gain=params['cohesion_gain'],
                               alignment_gain=params['alignment_gain'],
                               perception_radius=params['perception_radius'],
                               max_agents=params['max_agents'])

            velocity += command

            # Limit velodity to max_velocity
            velocity_norm = np.linalg.norm(velocity)
            if velocity_norm > params['max_velocity']:
                velocity = params['max_velocity'] * (velocity / velocity_norm)
            
            velocity[2] = 0  # Only move in xy-plane

            cf.cmdVelocityWorld(velocity, 0)

        t += 0.05
        timeHelper.sleep(0.05)

    print("Going back to initial positions")

    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop(remainValidMillisecs=100)
        cf.goTo(cf.initialPosition + np.array([0, 0, Z]), 0, 3.0)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
