#!/usr/bin/env python
import argparse
import rospy
import tf
from tf.msg import tfMessage
import PyKDL
from reconfigurable_transform_publisher.utils import print_transform

def get_authority(parent_frame, child_frame):
    authorities = {}
    def tf_callback(msg):
        authorities[msg.transforms[0].header.frame_id.strip('/'),msg.transforms[0].child_frame_id.strip('/')] = msg._connection_header['callerid']
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
        'reconfigurable_transform_publisher/transform_publisher to publish the transform.'\
        'This option should probably be used with -k')
    parser.add_argument('-m','--multistatic', metavar='MULTI_NODE_NAME', help='If set, configures a '\
        'lscr_tf_tools/multistatic_transform_publisher to publish the transform. '\
        'This option should probably be used with -k')
    parser.add_argument('-p','--print',choices=['launch','shell','both'], dest='print_transform', help='Prints the launch or shell '\
        'command for running a static_transform_publisher for this transform.')
    parser.add_argument('-w', '--wait', type=float, metavar='DURATION', default=5,
        help='How long to wait for the transform before giving up')

    args = parser.parse_args(rospy.myargv()[1:])
    args.frame_id = args.frame_id.strip('/')
    args.child_frame_id = args.child_frame_id.strip('/')

    rospy.init_node('make_static', anonymous=True)
    listener = tf.TransformListener()

    try:
        rospy.sleep(5)
        listener.waitForTransform(args.frame_id, args.child_frame_id, rospy.Time(), rospy.Duration(args.wait))
    except tf.Exception:
        rospy.logerr("Couldn't find transform between %s and %s after waiting %s seconds."
            % (args.frame_id, args.child_frame_id, args.wait))
        import sys
        sys.exit(1)

    try:
        trans, rot = listener.lookupTransform(args.frame_id, args.child_frame_id, rospy.Time())
    except tf.Exception as e:
        rospy.logerr(e.message)
        import sys
        sys.exit(1)

    if args.kill:
        import rosnode
        authority = get_authority(args.frame_id, args.child_frame_id)
        rosnode.kill_nodes([authority])

    if args.reconfig is not None:
        from reconfigurable_transform_publisher.cfg import TransformConfig
        import dynamic_reconfigure.client
        client = dynamic_reconfigure.client.Client(args.reconfig)
        roll, pitch, yaw = PyKDL.Rotation.Quaternion(*rot).GetRPY()
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

    if args.multistatic is not None:
        import lcsr_tf_tools.msg as lcsr_tf_tools
        from geometry_msgs.msg import TransformStamped
        tform = TransformStamped()
        tform.header.frame_id = args.frame_id
        tform.header.stamp = rospy.Time(0)

        tform.child_frame_id = args.child_frame_id

        tform.transform.translation.x = trans[0]
        tform.transform.translation.y = trans[1]
        tform.transform.translation.z = trans[2]

        tform.transform.rotation.x = rot[0]
        tform.transform.rotation.y = rot[1]
        tform.transform.rotation.z = rot[2]
        tform.transform.rotation.w = rot[3]

        static_tform = lcsr_tf_tools.StaticTransform()
        static_tform.header.stamp = rospy.Time.now()
        static_tform.transform = tform
        static_tform.publish_period = rospy.Duration(1E-3*10)

        set_pub = rospy.Publisher(args.multistatic+'/set_frame', lcsr_tf_tools.StaticTransform, latch=True)
        transform_ready = False
        rospy.sleep(2)
        while not transform_ready and not rospy.is_shutdown():
            set_pub.publish(static_tform)
            try:
                listener.lookupTransform(args.frame_id, args.child_frame_id, rospy.Time())
                transform_ready = True
            except tf.Exception as ex:
                rospy.logwarn("Multi static transform %s --> %s not being published yet: %s" % (tform.header.frame_id, tform.child_frame_id, str(ex)))

    if args.print_transform is not None:
        print_shell  = args.print_transform in ('shell, both')
        print_launch = args.print_transform in ('launch, both')
        print_transform(trans, rot, args.frame_id, args.child_frame_id, 10, shell=print_shell, launch=print_launch),
