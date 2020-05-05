from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, 'base_link', 'end_link')
q = kdl_kin.random_joint_angles()
pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics
if q_ik is not None:
    pose_sol = kdl_kin.forward(q_ik) # should equal pose
J = kdl_kin.jacobian(q)
print 'q:', q
print 'q_ik:', q_ik
print 'pose:', pose
if q_ik is not None:
    print 'pose_sol:', pose_sol
print 'J:', J