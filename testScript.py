# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1. 
2.
3.
'''
from spatialmath import SE3
from HW3_utils import FKHW3
from math import pi
import roboticstoolbox as rtb
import numpy as np
from spatialmath.base import *
#กำหนด ความยาวส่วนต่างๆของ Manipulator
d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = -0.093
d_6 = -0.082
q=[0.0,-pi/2.0,-0.2]
Trans =  transl(a_3+d_6,0.0,0.0) @ transl(0.0,d_5,0.0) @ transl(0.0,0.0,d_4) @ troty(-pi/2.0)
######################################
#สร้าง DH-Parameter ด้วย  Robotic Toolbox
robot=rtb.DHRobot(
        [
            rtb.RevoluteMDH(alpha = 0.0,a = 0.0,d = d_1,offset = pi), # DH-parameter จาก Frame 0 -> 1
            rtb.RevoluteMDH(alpha = pi/2.0,a =0.0 ,d = 0.0,offset = 0.0),# DH-parameter จาก Frame 1 -> 2
            rtb.RevoluteMDH(alpha = 0.0,a = a_2,d = 0.0,offset = 0.0)# DH-parameter จาก Frame 2 -> 3
        ],
        # Transformation Matrix จาก Frame 3 -> e
        tool=SE3([[0, 0, -1.0, a_3 + d_6],
    [0, 1.0, 0, d_5],
    [1.0, 0, 0, d_4],
    [0, 0, 0, 1.0]])
    )
print(robot)
#########################################
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def jacobiancheck(q):
    J_e=robot.jacob0(q) # คำนวน Jacobian Matrix ผ่าน Robotic Toolbox โดยใช้ [q1,q2,q3] ที่ให้
    return J_e
J_e=jacobiancheck(q)
print(jacobiancheck(q))
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def singularcheck(J_e):
    rank = np.linalg.matrix_rank(J_e) #ใช้ Robotic Toolbox เพื่อหา Rank (จำนวน Row หรือ Column ที่ ไม่สามารถมาจากผลลัพธ์ของ Row หรือ Column อื่นใน Matrix เดียวกัน)
    is_singular = rank < min(J_e.shape) #ตั้งเงื่อนไขเพื่อหา Singularity (ถ้า rank น้อยกว่า minimum ของ Jacobian Matrix)
    print(rank)
    print(is_singular)
    if is_singular==True: #ถ้าเป็น Singularity
        return 1
    else:
        return 0
singular_result=singularcheck(jacobiancheck(q))
print(singular_result)
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def effortcheck(q,w):
    tau = robot.pay(w,J=J_e,frame=0) #Robotic Toolbox คำนวนหส Tau โดย w=wrench, J=Jacobian Matrix, Frame=0 คือคำนวนโดยมี Frame 0 เป็น Reference 
    return tau
print(effortcheck(q,np.array([10,0,0,0,0,0])))
#==============================================================================================================#