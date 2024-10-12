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
