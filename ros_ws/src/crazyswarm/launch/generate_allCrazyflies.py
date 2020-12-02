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
    r = 1.5
    x0, y0 = 2.0, 0.0
    cf_ids = [3, 4, 5, 6, 7] 
    N = len(cf_ids)

    cfs = []
    for i, id in enumerate(cf_ids):
        x = x0 + r * np.cos(2 * np.pi * i / N)
        y = y0 + r * np.sin(2 * np.pi * i / N)
        cf = {'id': id,
              'channel': 80,
              'initialPosition': [float(x), float(y), 0.0],
              'type': 'defaultSingleMarker'}
        cfs.append(cf)

    yaml_dict = {'crazyflies': cfs}
    with open('allCrazyflies.yaml', 'w') as file:
        documents = yaml.dump(yaml_dict, file)
