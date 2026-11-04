from _msg import message
from builtin_interfaces.msg import Time
Header = message('Header', stamp=Time, frame_id='')
String = message('String', data='')
Float32 = message('Float32', data=0.0)
Float32MultiArray = message('Float32MultiArray', data=list)
ColorRGBA = message('ColorRGBA', r=0.0, g=0.0, b=0.0, a=0.0)
