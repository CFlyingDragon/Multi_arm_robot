from math import pi
import numpy as np
import numpy.matlib
import ik_UR5
from euler_to_rot import euler_to_rot
import time
from connect_ur import connect_robot

def connect1():
    global robot1
    robot1 = connect_robot('192.168.0.1')

def connect2():
    global robot2
    robot2 = connect_robot('192.168.0.2')

T_B1_W=np.matrix('-0.707 0.707 0 0;-0.707 -0.707 0 693;0 0 1 0;0 0 0 1')
T_B2_W=np.matrix('-0.707 0.707 0 0;-0.707 -0.707 0 -693;0 0 1 0;0 0 0 1')
T_E1_O=np.matrix('-1 0 0 0;0 0 -1 220;0 -1 0 0;0 0 0 1')
T_E2_O=np.matrix('1 0 0 0;0 0 1 -220;0 -1 0 0;0 0 0 1')

def move():

    N=1200
    k=np.linspace(-pi,pi,N)
    px=100*np.sin(k)
    py=np.zeros(N)
    pz=500+100*np.cos(k)
    kk=np.linspace(0,pi,N)
    rx=pi/1800+pi/18*np.sin(kk)
    ry=pi/1800+pi/18*np.sin(kk)
    rz=pi/1800+pi/18*np.sin(kk)

    T=np.matlib.empty((4,4))

    for i in range(N):

        rot=euler_to_rot([rx[i],ry[i],rz[i]])
        T[0:3,0:3]=rot
        T[0,3]=px[i]
        T[1,3]=py[i]
        T[2,3]=pz[i]
        T[3,:]=np.matrix('0 0 0 1')

        T1=np.matmul(T,T_E1_O)
        T1=np.matmul(np.linalg.inv(T_B1_W),T1)

        T2=np.matmul(T,T_E2_O)
        T2=np.matmul(np.linalg.inv(T_B2_W),T2)

        theta1=ik_UR5.ik_UR(T1)
        theta2=ik_UR5.ik_UR(T2)

        q1=[theta1[0,0],theta1[0,1],theta1[0,2],theta1[0,3],theta1[0,4],theta1[0,5]]
        data1 ='servoj('+str(q1)+',0,0,0.06,0.03,300)\n'

        q2=[theta2[0,0],theta2[0,1],theta2[0,2],theta2[0,3],theta2[0,4],theta2[0,5]]
        data2 ='servoj('+str(q2)+',0,0,0.06,0.03,300)\n'

        robot1.send(data1)
        robot2.send(data2)

        time.sleep(0.03)

def zero():

    T=np.matlib.empty((4,4))
    rot=euler_to_rot([pi/1800,pi/1800,pi/1800])
    T[0:3,0:3]=rot
    T[0:3,3]=np.matrix('0;0;400')
    T[3,:]=np.matrix('0 0 0 1')

    T1=np.matmul(T,T_E1_O)
    T1=np.matmul(np.linalg.inv(T_B1_W),T1)

    T2=np.matmul(T,T_E2_O)
    T2=np.matmul(np.linalg.inv(T_B2_W),T2)

    theta1=ik_UR5.ik_UR(T1)
    theta2=ik_UR5.ik_UR(T2)

    q1=[theta1[0,0],theta1[0,1],theta1[0,2],theta1[0,3],theta1[0,4],theta1[0,5]]
    data1 ='movej('+str(q1)+',a = 0.1,v=0.08)\n'

    q2=[theta2[0,0],theta2[0,1],theta2[0,2],theta2[0,3],theta2[0,4],theta2[0,5]]
    data2 ='movej('+str(q2)+',a = 0.1,v=0.08)\n'
    
    robot1.send(data1)
    robot2.send(data2)


if __name__=='__main__':
    try:
        connect1()
        connect2()
        time.sleep(5)
        zero()
        #move()
    except KeyboardInterrupt:
        zero()