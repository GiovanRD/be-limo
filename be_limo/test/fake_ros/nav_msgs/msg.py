from _msg import message
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
Path = message('Path', header=Header, poses=list)
Odometry = message('Odometry', header=Header, child_frame_id='', pose=PoseWithCovariance, twist=TwistWithCovariance)
