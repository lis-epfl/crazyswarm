import argparse
import numpy as np
import yaml

if __name__ == '__main__':
    # parser = argparse.ArgumentParser(description='Generates allCrazyflies.yaml')
    # parser.add_argument('r', type=float, help='Radius of circle')
    # parser.add_argument('x0', type=float, help='x-coordinate of center')
    # parser.add_argument('y0', type=float, help='y-coordinate of center')
    # parser.add_argument('N', type=int, help='Number of crazyflie')
    # args = parser.parse_args()

    # r = args.r
    # x0 = args.x0
    # y0 = args.y0
    # N = args.N
    N_x = 4
    N_y = 4

    d_x = 0.5
    d_y = 0.5

    X = np.arange(0, N_x) * d_x
    Y = np.arange(0, N_y) * d_y

    cfs = []
    id = 1
    channel = 80
    d_channel = 20

    for x in X:
        for y in Y:
            if id % 15 == 0:
                channel += d_channel
            cf = {'id': id,
                  'channel': channel,
                  'initialPosition': [float(x), float(y), 0.0],
                  'type': 'defaultSingleMarker'}
            cfs.append(cf)
            id += 1

    yaml_dict = {'crazyflies': cfs}
    with open('allCrazyflies.yaml', 'w') as file:
        documents = yaml.dump(yaml_dict, file)