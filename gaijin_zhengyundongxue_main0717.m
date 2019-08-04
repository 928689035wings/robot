clear;
clc;
syms K1 K2;
% left_guanjie=left_Forward_kinematic(45,80,100,60,60,60,250,300,100)
% right_guanjie=right_Forward_kinematic(45,80,50,60,60,60,250,2000,100)
% fuzhubi_guanjie=fuzhubi_Forward_kinematic(45,80,50,60,60,60,250,300,100)
n=10;
ft1=pi/2;
ft2=105*pi/180;
ft3=105*pi/180;
ft4=pi/2;
ft5=288*pi/180;
ft6=105*pi/180;
fl1=2200;
fl2=1200;
fl3=4026;
%生成原始矩阵
% lef=rand(n,4);
% right=rand(n,4);
% moduan=rand(n,4);
 moduan_rand=rand(n,20);
 dabi_rand=rand(n,20);
 dabi_moduan_rand=rand(n,20)
count_1=0;
%利用蒙德卡洛法生成末端点
 %变量范围需要重新考虑
 for i=1:n   
     %右臂正运动学
%     L0=0; theta3=0;L1=0;theta4=0;theta5=0;theta6=pi/2;theta7=pi/2;theta8=pi/2;L2=0;
    r_flag=1;  %1为右臂，-1为左臂
    right_R=rand(1,10);
    
    right_L0=770+800*right_R(1,10);
%     right_theta3=-pi/4+ft1*right_R(1,1); %正时 左
    right_theta3=pi/4+ft1*right_R(1,1); % 我们以45度的时候为初试位置 以转过的角度为范围 后期需要通过实际转移到theta的运算中
    %并未过多考虑实际关节之间的角度
    right_L1=fl1*right_R(1,7);
%     right_theta4=-pi/4+ft2*right_R(1,2);%为正时 仰 
     right_theta4=-pi/4+ft2*right_R(1,2); 
    right_theta5=-pi/3+ft3*right_R(1,3); %以中垂线为中线，左边为负
    right_theta6=-pi/4-ft4*right_R(1,4);%-45度为左边极限位置 顺时针-
    right_theta7=-225*pi/180+ft5*right_R(1,5); %右  以逆时针-225度为起点
    right_theta8=-95*pi/180+ft6*right_R(1,6);
     right_L2=fl2*right_R(1,8);
  

   right_guanjie = Forward_kinematicdh0620(r_flag,right_L0, right_theta3,right_L1,right_theta4,right_theta5,right_theta6,right_theta7,right_theta8,right_L2);
 %右臂取关节点 关节点1为定值坐标系
   right_guanjie1=right_guanjie(1,1:3);
   right_guanjie2=right_guanjie(2,1:3);
   right_guanjie3=right_guanjie(3,1:3);
   right_guanjie4=right_guanjie(4,1:3);
   right_guanjie5=right_guanjie(5,1:3);
   right_guanjie6=right_guanjie(6,1:3);
   right_guanjie7=right_guanjie(7,1:3);
   right_guanjie8=right_guanjie(8,1:3);
   right_guanjie9=right_guanjie(9,1:3);
   right_guanjie_N=right_guanjie(10,1:3);
   
%左钻臂   
%      L0=0; theta3=0;L1=0;theta4=0;theta5=0;theta6=pi/2;theta7=pi/2;theta8=pi/2;L2=0;

    l_flag=-1;
   left_R=rand(1,10);
   left_L0=770+800*left_R(1,10);
   left_theta3=pi/4+ft1*left_R(1,1);
   left_L1=fl1*left_R(1,7);
   left_theta4=-pi/4+ft2*left_R(1,2);
   left_theta5=-pi/3+ft3*left_R(1,3);
   left_theta6=-pi/4-ft4*left_R(1,4);
%    left_theta7=-pi/2+ft5*left_R(1,5);
 left_theta7=225*pi/180+ft5*left_R(1,5);%以-225度为起点
   left_theta8=-95*pi/180+ft6*left_R(1,6);
    left_L2=fl2*left_R(1,8);
%    left_L2=5784.9+fl2*left_R(1,8);
%    left_L3=5475.9+fl3*left_R(1,9);
 
  left_guanjie = Forward_kinematicdh0620(l_flag,left_L0,left_theta3,left_L1,left_theta4,left_theta5,left_theta6,left_theta7,left_theta8,left_L2);
%     left_bianliang(i,:)=[left_L0,left_theta1,left_theta2,left_theta3,left_theta4,left_theta5,left_theta6,left_L1,left_L2,left_L3];
%    left_guanjie = left_Forward_kinematic0408(left_L0,left_theta1,left_theta2,left_theta3,left_theta4,left_theta5,left_theta6,left_L1,left_L2,left_L3);%调用运动学模型
   left_guanjie1=left_guanjie(1,1:3);
   left_guanjie2=left_guanjie(2,1:3);
   left_guanjie3=left_guanjie(3,1:3);
   left_guanjie4=left_guanjie(4,1:3);
   left_guanjie5=left_guanjie(5,1:3);
   left_guanjie6=left_guanjie(6,1:3);
   left_guanjie7=left_guanjie(7,1:3);
   left_guanjie8=left_guanjie(8,1:3);
   left_guanjie9=left_guanjie(9,1:3);
   left_guanjie_N=left_guanjie(10,1:3);  
   
   %工作装置滑移部分
  L_dn=left_guanjie9+6028*(left_guanjie_N-left_guanjie9)/norm(left_guanjie_N-left_guanjie9);%(左关节后端点=前端点+6028*单位向量)
  R_dn=right_guanjie9+6028*(right_guanjie_N-right_guanjie9)/norm(right_guanjie_N-right_guanjie9);%

  
  %输入数据
  L_input=[l_flag left_L0 left_theta3 left_L1 left_theta4 left_theta5 left_theta6 left_theta7 left_theta8 left_L2]
  R_input=[r_flag right_L0  right_theta3 right_L1 right_theta4 right_theta5 right_theta6 right_theta7 right_theta8 right_L2]
  

 f_theta1=pi/2;
 f_theta2=-pi/2;
 F1=2000;
 fuzhubi_guanjie=fuzhubifoward(f_theta1,f_theta2,F1)
 f_guanjie1=fuzhubi_guanjie(1,1:3)
 f_guanjie2=fuzhubi_guanjie(1,4:6)
 
 
  
% %
% dabi_min_distance
% moduan_min_distance=main_distance_judgement0530(left_guanjie4,left_guanjie5,right_guanjie4,right_guanjie5);%两大臂之间防碰撞
% % Ldabi_moduan_min_distance
 moduan_min_distance=main_distance_judgement0530(left_guanjie9,L_dn,right_guanjie3,right_guanjie5) %大臂和前端
% moduan_min_distance=main_distance_judgement0530(left_guanjie3,left_guanjie5,right_guanjie9,R_dn) %大臂和前端
% % Rdabi_moduan_min_distance 
% moduan_min_distance=main_distance_judgement0530(left_guanjie9,L_dn,right_guanjie9,R_dn);
fu_Rdabi_min_distance=main_distance_judgement0530(right_guanjie4,right_guanjie5,f_guanjie1,f_guanjie2)
fu_Ldabi_min_distance=main_distance_judgement0530(left_guanjie4,left_guanjie5,f_guanjie1,f_guanjie2)
fu_R_moduan_min_distance=main_distance_judgement0530(right_guanjie9,R_dn,f_guanjie1,f_guanjie2)
fu_L_moduan_min_distance=main_distance_judgement0530(left_guanjie9,L_dn,f_guanjie1,f_guanjie2)

%g返回末端输入坐标关节信息
   moduan_rand(i,1)=i;
   moduan_rand(i,2)=left_guanjie9(1,1);%钎杆末端坐标 随油缸变化
   moduan_rand(i,3)=left_guanjie9(1,2);
   moduan_rand(i,4)=left_guanjie9(1,3);
   moduan_rand(i,5)=L_dn(1,1);%钎杆末端坐标 随油缸变化
   moduan_rand(i,6)=L_dn(1,2);
   moduan_rand(i,7)=L_dn(1,3);
   
   moduan_rand(i,8)=  right_guanjie7(1,1);
   moduan_rand(i,9)=  right_guanjie7(1,2);
   moduan_rand(i,10)= right_guanjie7(1,3);
   moduan_rand(i,11)= R_dn(1,1);
   moduan_rand(i,12)= R_dn(1,2);
   moduan_rand(i,13)= R_dn(1,3);
  %返回判断值
  %judge_t1,judge_t2,min_distance_1,min_distance_2,distance_flag,min_distance_result,dot_flag
   moduan_rand(i,14)=  moduan_min_distance(1,1);%judge_t1
   moduan_rand(i,15)=  moduan_min_distance(1,2);%judge_t2,
   moduan_rand(i,16)=  moduan_min_distance(1,3);%min_distance_1
   moduan_rand(i,17)=  moduan_min_distance(1,4);%min_distance_2
   moduan_rand(i,18)=  moduan_min_distance(1,5);%distance_flag
   moduan_rand(i,19)=  moduan_min_distance(1,6);%min_distance_result
   moduan_rand(i,20)=  moduan_min_distance(1,7);%dot_flag
    L_p_judge(1)=moduan_min_distance(1,8);%L_pengzhuangpoint(1)
    L_p_judge(2)=moduan_min_distance(1,9);%L_pengzhuangpoint(2)
    L_p_judge(3)=moduan_min_distance(1,10);%L_pengzhuangpoint(3)
    R_p_judge(1)=moduan_min_distance(1,11);%R_pengzhuangpoint(1)
    R_p_judge(2)=moduan_min_distance(1,12);%R_pengzhuangpoint(2)
    R_p_judge(3)=moduan_min_distance(1,13);%R_pengzhuangpoint(3)
 end



