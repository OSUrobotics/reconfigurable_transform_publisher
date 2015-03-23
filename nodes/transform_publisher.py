#!/usr/bin/env python
from reconfigurable_transform_publisher.cfg import TransformConfig
from reconfigurable_transform_publisher.utils import print_transform
from dynamic_reconfigure.server import Server
import rospy
import tf
from tf.transformations import euler_from_quaternion
import PyKDL
import sys
from threading import RLock
from easy_markers.interactive import InteractiveGenerator

config_lock = RLock()
broadcaster = None
trans, rot = None, None
parent, child = None, None
period = 100.0
inited = False

def set_from_config(config):
    global trans, rot, parent, child, inited
    trans = config['x'], config['y'], config['z']
    rot = PyKDL.Rotation.RPY(config['roll'], config['pitch'], config['yaw'])
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
            config['roll'], config['pitch'], config['yaw'] = rot.GetRPY()
            inited = True
        else:
            set_from_config(config)
        return config

def marker_cb(feedback):
    updates ={
     'x': feedback.pose.position.x,
     'y': feedback.pose.position.y,
     'z': feedback.pose.position.z
    }
    updates['yaw'], updates['pitch'], updates['roll'] = euler_from_quaternion(
        (
         feedback.pose.orientation.x,
         feedback.pose.orientation.y,
         feedback.pose.orientation.z,
         feedback.pose.orientation.w,
        ),
        axes='rzyx') # don't know why this is the right axis sequence...
    srv.update_configuration(updates)
    ig.server.doSetPose(update=None, name=feedback.marker_name, pose=feedback.pose, header=feedback.header)
    ig.server.applyChanges()


if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) == 10: # yaw-pitch-roll
        with config_lock:
            trans = [float(n) for n in argv[1:4]]
            y,p,r = [float(n) for n in argv[4:7]]
            rot = PyKDL.Rotation.RPY(y,p,r)
            parent = argv[7]
            child  = argv[8]
            period = float(argv[9])
    elif len(argv) == 11: # quaternion
        with config_lock:
            trans = [float(n) for n in argv[1:4]]
            rot   = PyKDL.Rotation.Quaternion(*[float(n) for n in argv[4:8]])
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
        
    rospy.init_node('reconfigurable_transform_publisher', anonymous=True)
    broadcaster = tf.TransformBroadcaster()
    srv = Server(TransformConfig, config_cb)

    ig = InteractiveGenerator()
    ig.makeMarker(controls=['move_x',
                            'rotate_x',
                            'move_y',
                            'rotate_y',
                            'move_z',
                            'rotate_z'],
                  frame=parent,
                  callback=marker_cb,
                  name=child,
                  pose=trans,
                  rot=rot.GetQuaternion())

    r = rospy.Rate(1/(period/1000.0))
    while not rospy.is_shutdown():
        with config_lock:
            broadcaster.sendTransform(trans, rot.GetQuaternion(), rospy.Time.now()+r.sleep_dur, child, parent)
        r.sleep()

    print_transform(trans, rot, parent, child, period)
