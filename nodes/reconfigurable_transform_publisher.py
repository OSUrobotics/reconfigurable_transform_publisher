#!/usr/bin/env python
PACKAGE = 'reconfigurable_transform_publisher'
import roslib; roslib.load_manifest(PACKAGE)
from dynamic_reconfigure.server import Server
from reconfigurable_transform_publisher.cfg import TransformConfig
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
from threading import RLock

config_lock = RLock()
broadcaster = None
trans, rot = None, None
parent, child = None, None
period = 100.0
inited = False

def set_from_config(config):
    global trans, rot, parent, child, inited
    trans = config['x'], config['y'], config['z']
    rot = quaternion_from_euler(config['roll'], config['pitch'], config['yaw'])
    parent = config['parent_frame']
    child = config['child_frame']
    inited = True


def config_cb(config, level):
    global trans, rot, parent, child, inited
    with config_lock:
        if not inited:
            config['child_frame'] = child
            config['parent_frame'] = parent
            config['x'], config['y'], config['z'] = trans
            config['yaw'], config['pitch'], config['roll'] = euler_from_quaternion(rot, axes='szyx')
            inited = True
        else:
            set_from_config(config)
        return config

if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) == 10: # yaw-pitch-roll
        with config_lock:
            trans = [float(n) for n in argv[1:4]]
            rot   = quaternion_from_euler(*list([float(n) for n in argv[4:7]]), axes='szyx')
            parent = argv[7]
            child  = argv[8]
            period = float(argv[9])
    elif len(argv) == 11: # quaternion
        with config_lock:
            trans = [float(n) for n in argv[1:4]]
            rot   = [float(n) for n in argv[4:8]]
            parent = argv[8]
            child  = argv[9]
            period = float(argv[10])
    elif len(argv) == 2 and argv[1] == '-h':
        sys.stderr.write('''Usage: reconfigurable_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) 
OR 
Usage: reconfigurable_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) 
''')
        sys.exit(1)
    else:
        set_from_config(TransformConfig.defaults)
        
    rospy.init_node(PACKAGE)
    broadcaster = tf.TransformBroadcaster()
    srv = Server(TransformConfig, config_cb)
    r = rospy.Rate(1/(period/1000.0))
    while not rospy.is_shutdown():
        with config_lock:
            broadcaster.sendTransform(trans, rot, rospy.Time.now()+r.sleep_dur, child, parent)
        r.sleep()
    transform_dict = dict()
    transform_dict['x'], transform_dict['y'],transform_dict['z'] = trans
    transform_dict['ax'], transform_dict['ay'], transform_dict['az'], transform_dict['aw'] = rot
    transform_dict['parent'] = parent
    transform_dict['child']  = child
    transform_dict['period'] = period
    print '\nrosrun tf static_transform_publisher %(x)s %(y)s %(z)s %(ax)s %(ay)s %(az)s %(aw)s %(parent)s %(child)s %(period)s' % transform_dict
    print '\n<node name="%(parent)s_to_%(child)s" pkg="tf" type="static_transform_publisher" args="%(x)s %(y)s %(z)s %(ax)s %(ay)s %(az)s %(aw)s %(parent)s %(child)s %(period)s"/>' % transform_dict
