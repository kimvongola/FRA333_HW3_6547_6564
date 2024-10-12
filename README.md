# FRA333_HW3_6547_6564  

#################################  
Testscript.py  
#################################  
Part กำหนดตัวแปร และ DH-Parameter  
กำหนดค่าและตัวแปรที่โจทย์ให้มาและ DH-Parameter เพื่อทำ Forward Kinematic  
d_1 = 0.0892  
a_2 = -0.425  
a_3 = -0.39243  
d_4 = 0.109  
d_5 = -0.093  
d_6 = -0.082  
q=[0.0,-pi/2.0,-0.2] (องศากำหนดเองเพื่่อใช้ทดสอบ Function)  
สร้าง DH-Parameter ผ่าน Robotic Toolbox เพื่อความถูกต้องและแม่นยำโดยใช้ตัวแปรที่กำหนดไว้  

robot=rtb.DHRobot(  
        [  
            rtb.RevoluteMDH(alpha = 0.0,a = 0.0,d = d_1,offset = pi), # DH-parameter จาก Frame 0 -> 1  
            rtb.RevoluteMDH(alpha = pi/2.0,a =0.0 ,d = 0.0,offset = 0.0),# DH-parameter จาก Frame 1 -> 2  
            rtb.RevoluteMDH(alpha = 0.0,a = a_2,d = 0.0,offset = 0.0)# DH-parameter จาก Frame 2 -> 3  
        ],  
        # Transformation Matrix จาก Frame 3 -> e  
        tool=SE3([[0, 0, -1.0, a_3 + d_6], # Translation ในแกน X  
    [0, 1.0, 0, d_5], #Translation ในแกน Y  
    [1.0, 0, 0, d_4], #Translation ในแกน Z  
    [0, 0, 0, 1.0]])  
    )  
print(robot)  
│  aⱼ₋₁  │ ⍺ⱼ₋₁  │     θⱼ     │   dⱼ   │  
├────────┼───────┼────────────┼────────┤  
│    0.0 │  0.0° │  q1 + 180° │ 0.0892 │  
│    0.0 │ 90.0° │         q2 │    0.0 │  
│ -0.425 │  0.0° │         q3 │    0.0 │  
└────────┴───────┴────────────┴────────┘  
  
┌──────┬─────────────────────────────────────────────────┐  
│ tool │ t = -0.47, -0.093, 0.11; rpy/xyz = 0°, -90°, 0° │  
└──────┴─────────────────────────────────────────────────┘  
///////// ข้อ 1 ///////////  
คำนวนหา Jacobian Matrix ผ่าน Robotic Toolobox jaco0() โดยเป็น Function ที่คำนวน Jacobian Matrix โดยมี Frame 0 (Base Frame) เป็นอ้างอิง  
def jacobiancheck(q):  
    J_e=robot.jacob0(q) # คำนวน Jacobian Matrix ผ่าน Robotic Toolbox โดยใช้ [q1,q2,q3] ที่ให้  
    return J_e  
J_e=jacobiancheck(q)  
//////// ข้อ 2 ///////////  
การคำนวนหา Singularity ทำได้โดยหา Rank ของ Jacobian Matrix แล้วเทียบกับ Minimum rank ของ Jacobian Matrix ถ้า Rank น้อยกว่า Minimum rank จะเป็น Singularity  
(การสูญเสียการควบคุมไป >= 1 DOF จะทำให้ Rank ลดลงกว่า Minimum rank ซึ่งสามารถควบคุมได้ครบทุก Dof(3 DoF))  
rank = np.linalg.matrix_rank(J_e) #ใช้ Robotic Toolbox เพื่อหา Rank (จำนวน Row หรือ Column ที่ ไม่สามารถมาจากผลลัพธ์ของ Row หรือ Column อื่นใน Matrix เดียวกัน)  
    is_singular = rank < min(J_e.shape) #ตั้งเงื่อนไขเพื่อหา Singularity (ถ้า rank น้อยกว่า minimum ของ Jacobian Matrix)  
    print(rank)  
    print(is_singular)  
    if is_singular==True: #ถ้าเป็น Singularity  
        return 1  
    else:  
        return 0  
singular_result=singularcheck(jacobiancheck(q))  
///////// ข้อ 3 ///////////  
การหา tau ผ่านสมการ tau=J_e.transpose @ w โดย J_e มาจากผลลัพธ์ jacob0() และ w มาจาก Wrench=[force,momentum] 
