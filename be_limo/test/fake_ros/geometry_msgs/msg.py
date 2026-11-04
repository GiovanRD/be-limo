from _msg import message
from std_msgs.msg import Header
Point = message('Point', x=0.0, y=0.0, z=0.0)
Vector3 = message('Vector3', x=0.0, y=0.0, z=0.0)
Quaternion = message('Quaternion', x=0.0, y=0.0, z=0.0, w=1.0)
Pose = message('Pose', position=Point, orientation=Quaternion)
PoseStamped = message('PoseStamped', header=Header, pose=Pose)
PoseArray = message('PoseArray', header=Header, poses=list)
PoseWithCovariance = message('PoseWithCovariance', pose=Pose, covariance=lambda: [0.0] * 36)
PoseWithCovarianceStamped = message('PoseWithCovarianceStamped', header=Header, pose=PoseWithCovariance)
Twist = message('Twist', linear=Vector3, angular=Vector3)
TwistWithCovariance = message('TwistWithCovariance', twist=Twist, covariance=lambda: [0.0] * 36)
Transform = message('Transform', translation=Vector3, rotation=Quaternion)
TransformStamped = message('TransformStamped', header=Header, child_frame_id='', transform=Transform)
