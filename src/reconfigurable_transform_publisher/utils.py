def print_transform(trans, rot, parent, child, period, shell=True, launch=True):
    transform_dict = dict()
    transform_dict['x'], transform_dict['y'],transform_dict['z'] = trans
    transform_dict['ax'], transform_dict['ay'], transform_dict['az'], transform_dict['aw'] = rot
    transform_dict['parent'] = parent
    transform_dict['child']  = child
    transform_dict['period'] = period

    if shell:
        print '\nrosrun tf static_transform_publisher %(x)s %(y)s %(z)s %(ax)s %(ay)s %(az)s %(aw)s %(parent)s %(child)s %(period)s' % transform_dict
    if launch:
        print '\n<node name="%(parent)s_to_%(child)s" pkg="tf" type="static_transform_publisher" args="%(x)s %(y)s %(z)s %(ax)s %(ay)s %(az)s %(aw)s %(parent)s %(child)s %(period)s"/>' % transform_dict
