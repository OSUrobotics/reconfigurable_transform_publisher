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
            trans = config['x'], config['y'], config['z']
            rot = quaternion_from_euler(config['roll'], config['pitch'], config['yaw'])
            parent = config['parent_frame']
            child = config['child_frame']
            
        return config

if __name__ == '__main__':
    if len(sys.argv) == 10: # yaw-pitch-roll
        with config_lock:
            trans = [float(n) for n in sys.argv[1:4]]
            rot   = quaternion_from_euler(*list(reversed([float(n) for n in sys.argv[4:7]])))
            parent = sys.argv[7]
            child  = sys.argv[8]
            period = float(sys.argv[9])
    elif len(sys.argv) == 11: # quaternion
        with config_lock:
            trans = [float(n) for n in sys.argv[1:4]]
            rot   = [float(n) for n in sys.argv[4:8]]
            parent = sys.argv[8]
            child  = sys.argv[9]
            period = float(sys.argv[10])
    else:
        sys.stderr.write('''Usage: reconfigurable_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) 
OR 
Usage: reconfigurable_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) 
''')
        sys.exit(1)
        
    rospy.init_node(PACKAGE)
    broadcaster = tf.TransformBroadcaster()
    srv = Server(TransformConfig, config_cb)
    r = rospy.Rate(1/(period/1000.0))
    while not rospy.is_shutdown():
        with config_lock:
            broadcaster.sendTransform(trans, rot, rospy.Time.now(), child, parent)
        r.sleep()