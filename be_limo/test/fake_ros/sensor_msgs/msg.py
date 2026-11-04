from _msg import message
from std_msgs.msg import Header
LaserScan = message('LaserScan', header=Header, angle_min=0.0, angle_max=0.0, angle_increment=0.0,
                    time_increment=0.0, scan_time=0.0, range_min=0.0, range_max=0.0, ranges=list, intensities=list)
Image = message('Image', header=Header, height=0, width=0, encoding='', is_bigendian=0, step=0, data=b'')
