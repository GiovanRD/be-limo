from _msg import message
from geometry_msgs.msg import Pose


class SpawnEntity:
    Request = message('SpawnEntity_Request', name='', xml='', robot_namespace='', initial_pose=Pose,
                      reference_frame='')


class DeleteEntity:
    Request = message('DeleteEntity_Request', name='')
