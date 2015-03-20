# reconfigurable_transform_publisher

reconfigurable_transform_publisher is like static_transform_publisher, except the transform can be updated at runtime.
A command line utility for manually sending a transform.
It will periodicaly republish the given transform. 

Usage: `reconfigurable_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds)`

OR 

Usage: `reconfigurable_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds)`

This transform is the transform of the coordinate frame from frame_id into the coordinate frame 
of the child_frame_id.  

The transform can be updated using either dynamic_reconfigure, or interactive markers.
