# FRA333_HW3_6547_6564  
#################################  
FRA_HW3_6547_6564
#################################  
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


   Part Denavit-Hartenberg Parameters หรือ DH-Table
	
	ในส่วนนี้จะเลือกใช้การดู Frame ด้วยการทำ DH-Table เเทนการทำในรูปเเบบ Forward Kinematic
	เนื่องจากมีความเเม่นยำมากกว่าการทำเเบบ Forward Kinematic อย่างเเน่นอน

		โดยจากการเคลื่อนที่ในเเต่ละเฟรม ได้ดังนี้
		
		Frame 0 -> 1 :
		
			ให้มีการเคลื่อนที่ในเเกน Z ด้วยระยะ d_1 เเละ มีการหมุน 180 องศา
			เเละ ในส่วนเเกน X ไม่มีการเปลี่ยนเเปลงใดๆ

		Frame 1 -> 2 :
		
			ให้มีการเคลื่อนที่ในเเกน X ด้วยระยะ 0 เเละ มีการหมุน 90 องศา
			เเละ ในส่วนเเกน Z ไม่มีการเปลี่ยนเเปลงใดๆ

		Frame 2 -> e :
		
			ให้มีการเคลื่อนที่ในเเกน X ด้วยระยะ a_2 ในทิศสวนทาง 
			เเละ ในส่วนเเกน Z ไม่มีการเปลี่ยนเเปลงใดๆ


	ได้ตาราง DH-Table ดังนี้

		┌────────┬───────┬────────────┬────────┐
		│  aⱼ₋₁       │ ⍺ⱼ-1       │        θⱼ         │     dⱼ      │
		├────────┼───────┼────────────┼────────┤
		│    0.0      │   0.0°    │  q1 + 180°        │   0.0892    │
		│    0.0      │  90.0°    │      q2              │    0.0      │
		│   -0.425    │   0.0°    │      q3              │    0.0      │
		└────────┴───────┴────────────┴────────┘




====================================<อธิบาย คำตอบข้อ1>===========================================

โดยในส่วนนี้เป็นการหา Jacobian ของ ณ จุด End-effector โดยมีโค้ด ดังนี้


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


	ในพาร์ทเเรกเป็นการประกาศสร้าง Matrix 6 * จำนวน Array ของ q ที่เป็น Matrix ว่าง
	
	โค้ดดังนี้
		J_e = np.zeros((6, len(q)))	



	เเละในพาร์ทที่สอง หา ผลต่างของ P หรือ P_e - P_i เเละ ทำการหา Z_i ด้วยวิธีการโค้ดดังนี้

		        Pn = p_e - P[:,i]

			โดยเเนวคิด คือ ตามโจทย์ที่กำหนดมา มี P_e มาให้ โดยใช้เป็นตั้งเทียบ
				เเละ  P_i ตามการเปลี่ยนเเปลงที่เกิดในเเต่ละเฟรมนั่นเอง



        		Z_i = R[:,2,i] 
			เเนวคิด คือ การนำ Matrix ของ R ในตำเเหน่งที่ต้องการ

	

		พาร์ทหา Linear Velocity 

			J_e[:3,i] = (np.cross(Z_i,Pn))
			
			โดยนำ Z_i เเละ Pn ที่ได้มาจากกำหนดตัวเเปรข้างต้น นำมา Cross กันเพื่อ
			หา Linear Velocity



		พาร์ทหา Angular Velocity

			J_e[3:, i] = Z_i
		


	ผลลัพธ์ของ end EffectorJacobian โดย q กำหนด เป็น [0.0, 0.0, 0.0] ตามลำดับ

			answer :
 				[[-1.09000000e-01  9.08449234e-01  4.83449234e-01]
 				[-3.10849887e-03 -1.11443290e-16 -5.93957990e-17]
 				[ 0.00000000e+00  3.10849887e-03  3.10849887e-03]
 				[ 0.00000000e+00  1.22464685e-16  1.22464685e-16]
 				[ 0.00000000e+00  1.00000000e+00  1.00000000e+00]
 				[ 1.00000000e+00  6.12323426e-17  6.12323426e-17]]


==================================<สิ้นสุด  คำตอบข้อ1>========================================





===================================<อธิบาย คำตอบข้อ 2>========================================

โดยส่วนนี้เป็นการหา Singularity โดยใช้รูปเเบบ Jacobian ที่ได้มาจากข้อ 1 ซึ่งสนใจเพียงส่วนของ Linear Velocity 
เพราะในการทำ Singularity เราสนใจเพียงการเคลื่อนย้ายที่เคลื่อนไป เเต่ไม่สนมุมที่มีการเปลี่ยนเเปลง หรือ Angular Velocity
โดยกำหนดด้วยโค้ดตามนี้ 
	

def checkSingularityHW3(q:list[float])->bool:
    J_e = endEffectorJacobianHW3(q)
    JReduce = J_e[:3,:]

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


	ในพาร์ทกำหนดสนใจเพียง Linear Velocity ตามโค้ดดังนี้
		
		JReduce = J_e[:3,:]


	ขั้นตอนต่อมาเป็นการหา Det ของ Jacobian_Reduce หรือ Manipulability เเละ นำไปหาขนาดของเวกเตอร์ด้วย Norm
	ตามโค้ดดังนี้
	
		m = np.linalg.det(JReduce)

	เเละพาร์ทการทำ Norm โดยในที่เนื่องจากเป็นเวกเตอร์หรือเลขตัวเดียว สามารถใช้ได้ทั้ง Norm() หรือ เพียงเเค่ abs() ก็ได้เช่นกัน
	ตามโค้ดดังนี้

		absM = abs(m)

    		normM = np.linalg.norm(m)
		


	ในขั้นตอนสุดท้ายเทียบตามเงื่อนไขการเปลี่ยนเเปลง เเละ กำหนดที่ได้มาจากโจทย์ไม่เกิน 0.0001
	โดยตามโค้ดดังนี้

		s = 0 # singularity 0 1

    		if normM < 0.001 :
        		s = 1

    		if s == 1:
        		flag = s

    		else:
        		flag = s

    		return flag


	ผลลัพธ์ checkSingularityHW3 เมื่อกำหนดให้ q มีค่า [0.0, -pi/2, -0.2]

			answer :
					1



===============================<สิ้นสุด  คำตอบข้อ 2>=============================================




====================================<อธิบาย คำตอบข้อ 3=======================================

ในส่วนการหา Tau ของระบบการทำงาน จะสนใจ 2 อย่าง คือ Jacobian ที่นำมา Transpose เเละ Wrench ตามลำดับ (Fx, Fy, Fz, Tx, Ty, Tz)
เเละจึงนำทั้งสองมา Cross กันเพื่อหา Tau ได้โค้ดตามนี้


def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e  = endEffectorJacobianHW3(q_initial)
    J_Trans = np.transpose(J_e)

    tau = J_Trans @ w

    return tau

	พาร์ทการทำ Transpose Jacobian ได้โค้ดตามนี้

		J_Trans = np.transpose(J_e)


	
	ส่วนสุดท้ายการหา Tau ได้โค้ดตามนี้

		tau = J_Trans @ w



	ผลลัพย์ computeEffortHW3 เมื่อกำหนดให้ q มีค่า [0.0, -pi/2, -0.2] เเละ rench_initial มีค่าตามี้ ตามลำดับ [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		
			answer :
				
				[-1.09 -0.93 -0.93]



=============================================<สิ้นสุด  คำตอบข้อ 3>======================================================


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
ผลลัพธ์ที่ได้จาก DHRobot

                │  aⱼ₋₁  │ ⍺ⱼ₋₁  │     θⱼ     │   dⱼ   │  
                ├────────┼───────┼────────────┼────────┤  
                │    0.0 │  0.0° │  q1 + 180° │ 0.0892 │  
                │    0.0 │ 90.0° │         q2 │    0.0 │  
                │ -0.425 │  0.0° │         q3 │    0.0 │  
                └────────┴───────┴────────────┴────────┘          
  
                ┌──────┬─────────────────────────────────────────────────┐  
                │ tool │ t = -0.47, -0.093, 0.11; rpy/xyz = 0°, -90°, 0° │ ผลลัพธ์ที่ได้จาก DHRobot  
                └──────┴─────────────────────────────────────────────────┘  
///////// ข้อ 1 ///////////  
คำนวนหา Jacobian Matrix ผ่าน Robotic Toolobox jaco0() โดยเป็น Function ที่คำนวน Jacobian Matrix โดยมี Frame 0 (Base Frame) เป็นอ้างอิง  

        J_e=robot.jacob0(q) # คำนวน Jacobian Matrix ผ่าน Robotic Toolbox โดยใช้ [q1,q2,q3] ที่ให้  
ผลลัพธ์ Jacobian Matrix ที่ได้จาก jaco0()  

        [-1.09000000e-01  9.08449234e-01  4.83449234e-01]  
         [-3.10849887e-03 -1.11443285e-16 -5.93957964e-17]  
         [ 3.59538873e-18  3.10849887e-03  3.10849887e-03]  
         [-1.34144565e-17  1.22464680e-16  1.22464680e-16]  
         [-6.62682453e-34  1.00000000e+00  1.00000000e+00]  
         [ 1.00000000e+00  6.12323400e-17  6.12323400e-17]  
 
//////// ข้อ 2 ///////////  
การคำนวนหา Singularity ทำได้โดยหา Rank ของ Jacobian Matrix แล้วเทียบกับ Minimum rank ของ Jacobian Matrix ถ้า Rank น้อยกว่า Minimum rank จะเป็น Singularity  
(การสูญเสียการควบคุมไป >= 1 DOF จะทำให้ Rank ลดลงกว่า Minimum rank ซึ่งสามารถควบคุมได้ครบทุก Dof(3 DoF))  

        rank = np.linalg.matrix_rank(J_e) #ใช้ Robotic Toolbox เพื่อหา Rank (จำนวน Row หรือ Column ที่ ไม่สามารถมาจากผลลัพธ์ของ Row หรือ Column อื่นใน Matrix เดียวกัน)  
            is_singular = rank < min(J_e.shape) #ตั้งเงื่อนไขเพื่อหา Singularity (ถ้า rank น้อยกว่า minimum ของ Jacobian Matrix)  
            if is_singular==True: #ถ้าเป็น Singularity  
                return 1  
            else:  
                return 0  
        singular_result=singularcheck(jacobiancheck(q))  
ผลลัพธ์ที่ได้จาก Function โดย 1=Singularity,0=Normal 

        0  
	#*****จากผลลัพธ์จะสังเกตเห็นได้ว่าผลลัพธ์จาก ||det(J_e)|| จะต่างจากการเทียบ Rank ของ Jacobian Matrix เนื่องจาก ||det(J_e)|| จะนำแค่ส่วน Linear Velocity Matrix มาใช้แต่การเทียบ Rank จะใช้ Jacobian Matrix ที่มีทั้ง Linear Velocity และ Angular Velocity ทำให้สรุปได้ว่า 3DOF-Manipulator จะอยู่ในสภาวะ Singularity ถ้าแขนหุ่นมีแค่ Linear Velocity แต่จะอยู่ในสภาวะ Normal ถ้าแขนหุ่นมีทั้ง Linear Velocity และ Angular Velocity*****#
///////// ข้อ 3 ///////////  
การหา tau ผ่านสมการ tau=J_e.transpose @ w โดย J_e มาจากผลลัพธ์ jacob0() และ w มาจาก Wrench=[force,momentum]  

    tau = robot.pay(w,J=J_e,frame=0) #Robotic Toolbox คำนวนหส Tau โดย w=wrench, J=Jacobian Matrix, Frame=0 คือคำนวนโดยมี Frame 0 เป็น Reference   
    print(effortcheck(q,np.array([10,0,0,0,0,0])))  # กำหนดให้ wrench=[10,0,0,0,0,0] สำหรับ Test Case  
ผลลัพธ์ที่ได้จาก Functin= [ 1.09,-9.08449234,-4.83449234]
