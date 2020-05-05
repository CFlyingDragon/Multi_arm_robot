import PyKDL

# create a vector
v = PyKDL.Vector(1,3,5)

# create a rotation from Roll Pitch, Yaw angles
r1 = PyKDL.Rotation.RPY(1.2, 3.4, 0)
#print r1

# create a rotation from XYZ Euler angles
r2 = PyKDL.Rotation.EulerZYX(0, 1, 0)

# create a rotation from a rotation matrix
r3 = PyKDL.Rotation(1,0,0, 0,1,0, 0,0,1)

# create a frame from a vector and a rotation
f = PyKDL.Frame(r2, v)

#create a wrench
wr = PyKDL.Wrench()
wr.force = PyKDL.Vector(0,1,2)
wr.torque = PyKDL.Vector(3,4,5)

print wr

#create a jacobian
jac = PyKDL.Jacobian(7)
print jac