import rospy
import numpy as np
import tf
from geometry_msgs.msg import Vector3, Vector3Stamped
import yaml


class CF():
    def __init__(self, id):
        self.id = id
        self.prefix = "/cf" + str(id)

        self.pos_publisher = rospy.Publisher(self.prefix + "/pos", Vector3, queue_size=1) 
        self.pos_msg = Vector3()
        self.vel_publisher = rospy.Publisher(self.prefix + "/vel", Vector3, queue_size=1) 
        self.vel_msg = Vector3()

    def publish_pos(self):
        tf.waitForTransform("/world", self.prefix, rospy.Time(0), rospy.Duration(10))
        position, quaternion = tf.lookupTransform("/world", self.prefix, rospy.Time(0))

        self.pos_msg.x = position[0]
        self.pos_msg.y = position[1]
        self.pos_msg.z = position[2]

        self.pos_publisher.publish(self.pos_msg)

    def publish_vel(self, dt):
        velocity = tf.lookupTwist(self.prefix, "/world", rospy.Time(0), rospy.Duration(dt))[0]

        self.vel_msg.x = velocity[0]
        self.vel_msg.y = velocity[1]
        self.vel_msg.z = velocity[2]

        self.vel_publisher.publish(self.vel_msg)


if __name__ == '__main__':

    crazyflies_yaml = "../launch/crazyflies.yaml"
    with open(crazyflies_yaml, 'r') as ymlfile:
            cfg = yaml.load(ymlfile)
    
    all_cfs = []
    for crazyflie in cfg["crazyflies"]:
        all_cfs.append(CF(int(crazyflie["id"])))

    dt = 0.1  # Time step for finite difference
    rate = 50  # Frequency of node
    
    rospy.init_node('pos_vel_publisher', anonymous=True)
    tf = tf.TransformListener()
    r = rospy.Rate(rate)

    rospy.sleep(2 * dt)  # Wait for the tf twist buffer to build up

    while not rospy.is_shutdown():
        for cf in all_cfs:
            cf.publish_pos()
            cf.publish_vel(dt)
        r.sleep()
