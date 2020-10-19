import numpy as np
from math import sin
from math import cos
from math import atan2
from math import sqrt
from math import pi

#p is x,y,z and euler angles R,P,Y
def ik_UR5(p):

    d1 = 89.2
    d4 = 109.3
    d5 = 94.75
    d6 = 82.5
    a2 = -425
    a3 = -392.25

    # d1 = 89.159
    # d4 = 109.15
    # d5 = 94.65
    # d6 = 82.3
    # a2 = -425
    # a3 = -392.25

    nx=cos(p[4])*cos(p[5])
    ny=sin(p[3])*sin(p[4])*cos(p[5])+cos(p[3])*sin(p[5])
    nz=-cos(p[3])*sin(p[4])*cos(p[5])+sin(p[3])*sin(p[5])

    ox=-cos(p[4])*sin(p[5])
    oy=-sin(p[3])*sin(p[4])*sin(p[5])+cos(p[3])*cos(p[5])
    oz=cos(p[3])*sin(p[4])*sin(p[5])+sin(p[3])*cos(p[5])

    ax=sin(p[4])
    ay=-sin(p[3])*cos(p[4])
    az=cos(p[3])*cos(p[4])

    px = p[0]; py = p[1]; pz = p[2]

    sym=np.matrix('1 1 1;1 1 -1;1 -1 1;1 -1 -1;-1 1 1;-1 1 -1;-1 -1 1;-1 -1 -1')

    theta=np.empty((8,6))

    NUM=0
    for i in range(8):
    
        theta1 = atan2(d6*ay-py,d6*ax-px) - atan2(d4,sym[i,0]*sqrt(pow(d6*ax-px,2) + pow(d6*ay-py,2) - d4*d4))

        sin5=sym[i,1]*sqrt(1-pow(ax*sin(theta1)-ay*cos(theta1),2))

        theta5 = atan2(sin5,sin(theta1) * ax - cos(theta1) * ay)

        theta6 = atan2( (-sin(theta1) * ox + cos(theta1) * oy)/sin5,\
                     (sin(theta1) * nx - cos(theta1) * ny )/sin5)

        A=d5*(sin(theta6)*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6)*(ox*cos(theta1)+oy*sin(theta1)))\
                -d6*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1)

        B=pz-d1-az*d6+d5*(oz*cos(theta6)+nz*sin(theta6))

        theta3=atan2(sym[i,2]*sqrt(4*a2*a2*a3*a3-pow(A*A+B*B-a2*a2-a3*a3,2)),A*A+B*B-a2*a2-a3*a3)

        theta2=atan2((a3*cos(theta3)+a2)*B-a3*sin(theta3)*A,(a3*cos(theta3)+a2)*A+a3*sin(theta3)*B)

        C=(nx*cos(theta1)+ny*sin(theta1))*cos(theta6)*cos(theta5)-(ox*cos(theta1)+oy*sin(theta1))*sin(theta6)*cos(theta5)\
                -(ax*cos(theta1)+ay*sin(theta1))*sin(theta5)

        D=nz*cos(theta6)*cos(theta5)-oz*sin(theta6)*cos(theta5)-az*sin(theta5)

        theta4=atan2(D,C)-theta2-theta3

        theta[NUM,:] = [theta1,theta2,theta3,theta4,theta5,theta6]

        NUM=NUM+1

    return theta


def ik_UR(T):

    d1 = 89.2
    d4 = 109.3
    d5 = 94.75
    d6 = 82.5
    a2 = -425
    a3 = -392.25

    # d1 = 89.159
    # d4 = 109.15
    # d5 = 94.65
    # d6 = 82.3
    # a2 = -425
    # a3 = -392.25

    nx = T[0,0]; ny = T[1,0]; nz = T[2,0]
    ox = T[0,1]; oy = T[1,1]; oz = T[2,1]
    ax = T[0,2]; ay = T[1,2]; az = T[2,2]
    px = T[0,3]; py = T[1,3]; pz = T[2,3]

    sym=np.matrix('1 1 1;1 1 -1;1 -1 1;1 -1 -1;-1 1 1;-1 1 -1;-1 -1 1;-1 -1 -1')

    theta=np.empty((8,6))

    NUM=0
    for i in range(8):
    
        theta1 = atan2(d6*ay-py,d6*ax-px) - atan2(d4,sym[i,0]*sqrt(pow(d6*ax-px,2) + pow(d6*ay-py,2) - d4*d4))

        sin5=sym[i,1]*sqrt(1-pow(ax*sin(theta1)-ay*cos(theta1),2))

        theta5 = atan2(sin5,sin(theta1) * ax - cos(theta1) * ay)

        theta6 = atan2( (-sin(theta1) * ox + cos(theta1) * oy)/sin5,\
                     (sin(theta1) * nx - cos(theta1) * ny )/sin5)

        A=d5*(sin(theta6)*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6)*(ox*cos(theta1)+oy*sin(theta1)))\
                -d6*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1)

        B=pz-d1-az*d6+d5*(oz*cos(theta6)+nz*sin(theta6))

        theta3=atan2(sym[i,2]*sqrt(4*a2*a2*a3*a3-pow(A*A+B*B-a2*a2-a3*a3,2)),A*A+B*B-a2*a2-a3*a3)

        theta2=atan2((a3*cos(theta3)+a2)*B-a3*sin(theta3)*A,(a3*cos(theta3)+a2)*A+a3*sin(theta3)*B)

        C=(nx*cos(theta1)+ny*sin(theta1))*cos(theta6)*cos(theta5)-(ox*cos(theta1)+oy*sin(theta1))*sin(theta6)*cos(theta5)\
                -(ax*cos(theta1)+ay*sin(theta1))*sin(theta5)

        D=nz*cos(theta6)*cos(theta5)-oz*sin(theta6)*cos(theta5)-az*sin(theta5)

        theta4=atan2(D,C)-theta2-theta3

        theta[NUM,:] = [theta1,theta2,theta3,theta4,theta5,theta6]

        NUM=NUM+1

    return theta
    
    
if __name__=="__main__":
    p=[290,-210,221,pi,0,-pi/4]
    theta=ik_UR5(p)
    print(theta)