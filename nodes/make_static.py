#!/usr/bin/env python
import argparse
import rospy
import tf
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from reconfigurable_transform_publisher.utils import print_transform
import dynamic_reconfigure.client

def get_authority(parent_frame, child_frame):
    authorities = {}
    def tf_callback(msg):
        authorities[msg.transforms[0].header.frame_id,msg.transforms[0].child_frame_id] = msg._connection_header['callerid']
    rospy.Subscriber('tf', tfMessage, tf_callback)
    while not rospy.is_shutdown() and (parent_frame, child_frame) not in authorities:
        rospy.sleep(0.01)
    return authorities[parent_frame, child_frame]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Make a transform static. Looks up the transform '\
        'between frame_id and child_frame_id and provides several options for making it static.')
    parser.add_argument('frame_id')
    parser.add_argument('child_frame_id')
    parser.add_argument('-k','--kill', action='store_true', help='Kill the node publishing the transform')
    parser.add_argument('-r','--reconfig', metavar='NODE_NAME', help='If set, configures a '\
        'reconfigurable_transform_publisher/transform_publisher to publish the transform. '\
        'This option should probably be used with -k')
    parser.add_argument('-p','--print',choices=['launch','shell','both'], dest='print_transform', help='Prints the launch or shell '\
        'command for running a static_transform_publisher for this transform.')

    args = parser.parse_args()

    rospy.init_node('make_static', anonymous=True)
    listener = tf.TransformListener()
    listener.waitForTransform(args.frame_id, args.child_frame_id, rospy.Time(), rospy.Duration(5))
    trans, rot = listener.lookupTransform(args.frame_id, args.child_frame_id, rospy.Time())

    if args.kill:
        import rosnode
        authority = get_authority(args.frame_id, args.child_frame_id)
        rosnode.kill_nodes([authority])

    if args.reconfig is not None:
        from reconfigurable_transform_publisher.cfg import TransformConfig
        from dynamic_reconfigure.server import Server
        client = dynamic_reconfigure.client.Client(args.reconfig)

        yaw, pitch, roll = euler_from_quaternion(rot)
        client.update_configuration({
            'parent_frame' : args.frame_id,
            'child_frame' : args.child_frame_id,
            'x' : trans[0],
            'y' : trans[1],
            'z' : trans[2],
            'yaw' : yaw,
            'pitch' : pitch,
            'roll' : roll
        })

    if args.print_transform is not None:
        print_shell  = args.print_transform in ('shell, both')
        print_launch = args.print_transform in ('launch, both')
        print_transform(trans, rot, args.frame_id, args.child_frame_id, 10, shell=print_shell, launch=print_launch),