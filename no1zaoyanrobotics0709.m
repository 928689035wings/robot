%第一种DH凿岩台车建模
%0619增加偏移量
clc;
clear;
startup_rvc;
a1=215;
L0=770;
R_flag=1126;
theta1_norm=1.2580-acos((3764374-1540*L0-L0*L0)/3776630)
% theta1_norm=20*pi/180;
% 
% R_theta1=-theta1_norm;  %必须为负数
% R_theta2=theta1_norm;
% R_theta3=pi/2+0*pi/180;
% % R_theta3=pi/2+0*pi/180;
% R_theta4=0-0*pi/180;
% % R_theta5=0+45*pi/180;
% R_theta5=-pi/3+60*pi/180;
% R_theta6=-pi/4-45*pi/180;
% R_theta7=225*pi/180+180*pi/180;
% R_theta8=0;
% 
R_theta1=-theta1_norm;  %必须为负数
R_theta2=theta1_norm;
R_theta3=pi/2;
R_theta4=0;
R_theta5=0;
R_theta6=-pi/2;
R_theta7=-pi/2;
R_theta8=0;

 W=[-15 +15  -15 +15 -15 +15]*800;
%定值
R_theta9=0;
R_theta10=0;

a5=4000;   %a2 5250~7450
a6=145;  %a3  145
a7=240;% a5 240
a10=0  %a7

d6=248;%d4
d7=900;%d5
d8=1000;%0-2025
d9=2000;%5800-7000
d10=6000 %d7 5500-9500
% q=[R_theta1 R_theta2 R_theta3 R_theta4 R_theta5]



% 
R_q=[R_theta1 R_theta2 R_theta3 R_theta4 R_theta5 R_theta6 R_theta7 R_theta8 R_theta9 R_theta10];
% %    ];
% %     R_theta6 R_theta7 R_theta8];
% %          R_theta di  ai-1  ar i-1
R(1)=Link([R_theta1 0 R_flag pi/2],'modified')
R(2)=Link([R_theta2 0 1182 0],'modified')
R(3)=Link([R_theta3 2000 0 -pi/2],'modified')

% R(4)=Link([0 0 a4 -pi/2],'modified')
% %  a4 =215;
% a4=0;
a4=215;%
R(4)=Link([R_theta4 0 a4 -pi/2],'modified')
L1=1200;a5=5250+L1;
R(5)=Link([R_theta5 0 a5 0],'modified')
a6 = 144;d6 = 248;%
R(6)=Link([R_theta6 d6 a6 pi/2],'modified')

R(7)=Link([R_theta7 d7 a7 -pi/2],'modified')
a8 = 240;d8 = 580;
% R(8)=Link([R_theta8 d8 a8 pi/2 ],'modified')
R(8)=Link([R_theta8 d8 a8 -pi/2 ],'modified')
L2=200;d9=2440+L2;
R(9)=Link([R_theta9 d9 0 pi/2],'modified') %此处若为1设定为移动坐标 则无法表示出来
R(10)=Link([R_theta10 d10 a10 0 1],'modified')

% 
% %左边机械臂
L_theta1=pi+theta1_norm;
L_theta2=-L_theta1;
L_flag=-1126
L_theta3=45*pi/180;
L_theta4=0;
L_theta5=0
L_q=[L_theta1 L_theta2 R_theta3 R_theta4 R_theta5 R_theta6 R_theta7 R_theta8 R_theta9 R_theta10];
% L_q=[L_theta1 L_theta2 L_theta3]
% L_q=[L_theta1 L_theta2]
%          R_theta di  ai-1  ar i-1
L(1)=Link([R_theta1 0 L_flag pi/2],'modified')
L(2)=Link([R_theta2 0 1182 0],'modified')
L(3)=Link([R_theta3 2000 0 -pi/2],'modified')
% L(4)=Link([0 0 0 -pi/2],'modified')

L(4)=Link([R_theta4 0 0 -pi/2],'modified')
L(5)=Link([R_theta5 0 a5 0],'modified')

L(6)=Link([R_theta6 0 0 pi/2],'modified')
L(7)=Link([R_theta7 d7 a7 -pi/2],'modified')
L(8)=Link([R_theta8 d8 a8 pi/2 ],'modified')
L(9)=Link([R_theta9 d9 0 -pi/2],'modified') %此处若为1设定为移动坐标 则无法表示出来
L(10)=Link([R_theta10 d10 a10 0 1],'modified')
% 
f_theta1=pi/2;
f_theta2=-pi/2; %-pi/4 pi/2
f_theta3=-pi/2;
F1=2000
f5 = 5250+F1;
T_F=[f_theta1 f_theta2 f_theta3]
F(1)=Link([f_theta1,1083,0,0],'modified')
F(2)=Link([f_theta2,0,0,-pi/2],'modified')
a5 = 5250+L1;
F(3)=Link([f_theta3,a5,0,-pi/2],'modified')
zaoyan_robot_F=SerialLink(F,'name','zaoyan_f')
zaoyan_robot_F.plot(T_F,'workspace', W);
F_forward_kinematics=zaoyan_robot_F.fkine(T_F)


zaoyan_robot(1)=SerialLink(R,'name','zaoyan_R');
zaoyan_robot(1).plot(R_q,'workspace', W);
 hold on
zaoyan_robot(2)=SerialLink(L,'name','zaoyan_L');
zaoyan_robot(2).plot(L_q,'workspace', W);
hold on
zaoyan_robot(3)=SerialLink(F,'name','zaoyan_f');
zaoyan_robot(3).plot(T_F,'workspace', W);

% R_forward_kinematics=zaoyan_robot(1).fkine(R_q) %末端姿态和坐标
% L_forward_kinematics=L_zaoyan_robot.fkine(L_q)
 
