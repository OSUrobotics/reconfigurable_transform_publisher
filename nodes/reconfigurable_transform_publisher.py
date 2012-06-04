#!/usr/bin/env python
PACKAGE = 'reconfigurable_transform_publisher'
import roslib; roslib.load_manifest(PACKAGE)
from dynamic_reconfigure.server import Server
from reconfigurable_transform_publisher.cfg import TransformConfig
import rospy
import tf
from tf.transformations import quaternion_from_euler

def config_cb(config, level):
    print 'got config'
    print config
    return config

if __name__ == '__main__':
    print 1
    rospy.init_node(PACKAGE)
    print 2
    srv = Server(TransformConfig, config_cb)
    print 3
    rospy.spin()