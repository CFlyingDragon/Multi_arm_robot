#import roslib; roslib.load_manifest("hrl_geom")
from hrl_geom.pose_converter import PoseConv
pos_euler = [[0.1, 0.2, 0.3], [0.3, 0., 0.]]
twist_msg = PoseConv.to_twist_msg(pos_euler)
print twist_msg
#linear:
#  x: 0.1
#  y: 0.2
#  z: 0.3
#angular:
#  x: 0.3
#  y: 0.0
#  z: 0.0
homo_mat = PoseConv.to_homo_mat(twist_msg)
print homo_mat
#[[ 1.          0.          0.          0.1       ]
# [ 0.          0.95533649 -0.29552021  0.2       ]
# [-0.          0.29552021  0.95533649  0.3       ]
# [ 0.          0.          0.          1.        ]]
[pos, rot] = homo_mat[:3,3], homo_mat[:3,:3]
tf_stamped_msg = PoseConv.to_tf_stamped_msg("base_link", pos, rot)
print tf_stamped_msg
#header:
#  seq: 0
#  stamp:
#    secs: 1341611677
#    nsecs: 579762935
#  frame_id: base_link
#child_frame_id: ''
#transform:
#  translation:
#    x: 0.1
#    y: 0.2
#    z: 0.3
#  rotation:
#    x: 0.149438132474
#    y: 0.0
#    z: 0.0
#    w: 0.988771077936
new_pos_euler = PoseConv.to_pos_euler(tf_stamped_msg)
print new_pos_euler
#([0.10000000000000001, 0.20000000000000001, 0.29999999999999999], (0.30000000000000004, -0.0, 0.0))