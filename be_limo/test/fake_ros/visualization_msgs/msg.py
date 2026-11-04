from _msg import message
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3
from builtin_interfaces.msg import Duration
Marker = message('Marker', header=Header, ns='', id=0, type=0, action=0, pose=Pose, scale=Vector3,
                 color=ColorRGBA, lifetime=Duration, points=list, text='')
for _k, _v in dict(ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, LINE_STRIP=4, LINE_LIST=5, TEXT_VIEW_FACING=9).items():
    setattr(Marker, _k, _v)
MarkerArray = message('MarkerArray', markers=list)
