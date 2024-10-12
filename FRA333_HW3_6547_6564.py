# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย

from HW3_utils import FKHW3
from spatialmath import SE3
import numpy as np
import math
from math import pi

import roboticstoolbox as rtb
from spatialmath.base import *
from scipy.stats import norm

print(np.__version__)

'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1. ภวดล_6547
2. กษิดิษฐ์_6564
'''


####### ทำการประกาศตัวเเปร ######

d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = -0.093
d_6 = -0.082

q_initial = np.array([0.0, 0.0, 0.0])
q_singularity = np.array([0.0, -pi/2, -0.2])
Wrench_initial = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])


'''
    Part Transformation
        การเคลื่อนย้ายจาก Frame ที่ 3 ไปยังจุดเสุดท้าย End-Effector
        โดยในการเคลื่อนย้ายสามารถสรุปได้ทั้งนี้
            การเคลื่อนย้ายในเเกน X :

                Trans_X = a_3 + d_6 

            การเคลื่อนย้ายในเเกน Y :
                
                Trans_Y = -d_5

            การเคลื่อนย้ายในเเกน Z :

                Trans_Z = d_4

        เเละ มีการหมุนของเเกน Y ดังนี้

                Rot_Y = -90 degree


        สุดท้ายนำมาทำ Transformation Matrix สู่ตำเเหน่ง End-Effector

'''

Trans =  transl(a_3+d_6,0.0,0.0) @ transl(0.0,d_5,0.0) @ transl(0.0,0.0,d_4) @ troty(-pi/2.0)
print("Transformation Matrix : \n", Trans)
print("")


'''
    Part DH-TABLE
'''
robot = rtb.DHRobot(
    [

       rtb.RevoluteMDH(a=0.0, alpha=0.0, d=d_1, offset=pi), #frame 1
       rtb.RevoluteMDH(a=0.0, alpha=pi/2.0, d=0.0, offset=0.0), #frame 1
       rtb.RevoluteMDH(a=a_2, alpha=0.0, d=0.0, offset=0.0) #frame 1 

    ], tool = SE3(Trans), name="3 DOF"
)

print("DH-Table : \n")
print(robot, "\r\n")

###############################







#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    R, P, R_e, p_e = FKHW3(q)
    J_e = np.zeros((6, len(q)))

    for i in range(len(q)):

        Pn = p_e - P[:,i]
        Z_i = R[:,2,i] 

        # พาร์ดหา Velocity ต่างๆ
        J_e[:3,i] = (np.cross(Z_i,Pn)) # Linear Velocity
        J_e[3:, i] = Z_i # Angular Velocity

    return J_e
    
#==============================================================================================================#

#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J_e = endEffectorJacobianHW3(q)
    JReduce = J_e[:3,:]
    print(JReduce)

    m = np.linalg.det(JReduce)

    absM = abs(m)
    normM = np.linalg.norm(m)

    s = 0 # singularity 0 1

    if normM < 0.001 :
        s = 1

    if s == 1:
        flag = s

    else:
        flag = s

    return flag

#==============================================================================================================#

#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e  = endEffectorJacobianHW3(q_initial)
    J_Trans = np.transpose(J_e)

    tau = J_Trans @ w

    return tau

#==============================================================================================================#

print("#############################################################")
print(" 1. endEffectorJacobianHW3 \n")
J_e = endEffectorJacobianHW3(q_singularity)
print("answer : \n", J_e, "\r\n")
print("#############################################################")


print("2. checkSingularityHW3 \n")
flag = checkSingularityHW3(q_singularity)
print("answer : \n", flag, "\r\n")
print("#############################################################")

print("3. computeEffortHW3 \r\n")
tau = computeEffortHW3(q_initial, Wrench_initial)
print("answer : \n", tau, "\r\n")

print("#############################################################")

