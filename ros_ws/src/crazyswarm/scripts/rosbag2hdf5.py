import os
import glob
import argparse
import rosbag
import h5py
import numpy as np


def read_bag(bag):
    """Read rosbag file and store data in a dict."""

    pos_data = {}
    data = {}

    for topic, msg, t in bag.read_messages():

        if topic == '/tf':
            # Position from motion capture system
            # Use time stamp from tf instead of time stamp from message
            t_secs = msg.transforms[0].header.stamp.secs
            t_nsecs = msg.transforms[0].header.stamp.nsecs
            t = t_secs + t_nsecs / 1e9  # Float time in seconds

            cf_id = msg.transforms[0].child_frame_id
            x = msg.transforms[0].transform.translation.x
            y = msg.transforms[0].transform.translation.y
            z = msg.transforms[0].transform.translation.z

            # Create dict for cf_id if non-existent
            if cf_id not in data:
                data[cf_id] = {}
            # Add pos key if non-existent
            if 'pos' not in data[cf_id]:
                data[cf_id]['pos'] = []
            # Append position data
            data[cf_id]['pos'].append([t, x, y, z])

        if '/cf' in topic:
            cf_id, cmd_name = topic.split('/')[1:3]

            # Time stamp has same structure for every command
            t_secs = msg.header.stamp.secs
            t_nsecs = msg.header.stamp.nsecs
            t = t_secs + t_nsecs / 1e9  # Float time in seconds

            # Create dict for cf_id if non-existent
            if cf_id not in data:
                data[cf_id] = {}
            # Initialize pos_ref, vel_ref, acc_ref if non-existent
            if 'pos_ref' not in data[cf_id]:
                data[cf_id]['pos_ref'] = []
            if 'vel_ref' not in data[cf_id]:
                data[cf_id]['vel_ref'] = []
            if 'acc_ref' not in data[cf_id]:
                data[cf_id]['acc_ref'] = []

            # Get position, velocity, acceleration reference values from command
            if cmd_name == 'cmd_position':
                pos_ref = [t, msg.x, msg.y, msg.z]
                data[cf_id]['pos_ref'].append(pos_ref)
            elif cmd_name == 'cmd_full_state':
                pos_ref = [t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                vel_ref = [t, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
                acc_ref = [t, msg.acc.x, msg.acc.y, msg.acc.z]
                data[cf_id]['pos_ref'].append(pos_ref)
                data[cf_id]['vel_ref'].append(vel_ref)
                data[cf_id]['acc_ref'].append(acc_ref)
            elif cmd_name == 'cmd_velocity_world':
                vel_ref = [t, msg.vel.x, msg.vel.y, msg.vel.z]
                data[cf_id]['vel_ref'].append(vel_ref)
    
    return data

def save_as_h5(data, file_path):
    """Save data dict to h5 file.
    `data` is a nested dictionary which contains a dict for every crazyflie. The latter stores
    information such as position, reference position, reference velocity, etc.
    """
    hf = h5py.File(file_path, 'w')

    # Write to hdf5 file
    for cf_id, cf_data in data.items():
        for key, value in data[cf_id].items():
            value_np = np.array(value)  # Convert to numpy
            hf.create_dataset(cf_id + '/' + key, data=value_np)

    hf.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Rosbag to hdf5 converter')
    parser.add_argument('bag', 
                        type=str, 
                        help="Path to .bag file or directory containing bag files")
    args = parser.parse_args()

    bag_files = []
    if os.path.isfile(args.bag):
        bag_files.append(args.bag)
    elif os.path.isdir(args.bag):
        search_path = os.path.join(args.bag, '*.bag')
        bag_files.extend(glob.glob(search_path))
    else:
        print("WARNING: {} is neither bagfile nor directory.".format(args.bag))

    for bag_file in bag_files:
        bag_path, _ = os.path.splitext(bag_file)
        h5file_path = bag_path + '.h5' 

        print("Converting {} ...".format(bag_file))
        bag = rosbag.Bag(bag_file, 'r')
        data = read_bag(bag)
        save_as_h5(data, h5file_path)