from math import sin
from math import cos
from math import pi,atan2,sqrt
import numpy.matlib

def euler_to_rot(p):
    #motion euler angle x,y,z.(Rx*Ry*Rz)

    rot=numpy.matlib.zeros((3,3))

    rot[0,0]=cos(p[1])*cos(p[2])
    rot[1,0]=sin(p[0])*sin(p[1])*cos(p[2])+cos(p[0])*sin(p[2])
    rot[2,0]=-cos(p[0])*sin(p[1])*cos(p[2])+sin(p[0])*sin(p[2])

    rot[0,1]=-cos(p[1])*sin(p[2])
    rot[1,1]=-sin(p[0])*sin(p[1])*sin(p[2])+cos(p[0])*cos(p[2])
    rot[2,1]=cos(p[0])*sin(p[1])*sin(p[2])+sin(p[0])*cos(p[2])

    rot[0,2]=sin(p[1])
    rot[1,2]=-sin(p[0])*cos(p[1])
    rot[2,2]=cos(p[0])*cos(p[1])

    return rot

if __name__=="__main__":
    p=[pi,0,0]
    T = euler_to_rot(p)
    print(T)

    nx = T[0,0]
    ny = T[1,0]
    nz = T[2,0]
    ox = T[0,1]
    oy = T[1,1]
    oz = T[2,1]
    ax = T[0,2]
    ay = T[1,2]
    az = T[2,2]

    rx = atan2(-ay,az)
    ry = atan2(ax,sqrt(nx*nx+ox*ox))
    rz = atan2(-ox,nx)

    print([rx,ry,rz])