function [Joint] = Forward_kinematicdh0620(lr_flag,L0, theta3,L1,theta4,theta5,theta6,theta7,theta8,L2)%,L3
%6.1号修改在DH左右翼展角度不同取值问题，解决数据不合常理，图像方向不同
%未解决问题 坐标显示翼展坐标往上 与robotic显示不符合
%第五个关节处出现长距离关节，与robotic和事实不符
%0620重新比对了DH与robotics模型
%翼展部分

% x_flag=1126*lr_flag % flag=1右臂 左臂-1  初试相对坐标 两臂展为中心  平行翼展面为Z轴 右翼展为X正，垂直面为Y需要确认
theta1_norm=1.2580-acos((3764374-1540*L0-L0*L0)/3776630)
theta1=-theta1_norm;  %必须为负数
% 初值
x_flag=0;
arpher8=0;
L_p_judge=[0 0 0];
R_p_judge=[0 0 0];
theta2=45*pi/180;


if lr_flag==1 %右臂
    x_flag=1126;
    arpher8=-pi/2;
    arpher9=pi/2;
    theta1=-theta1_norm;  %必须为负数
    theta2=theta1_norm;
elseif lr_flag==-1 %如果lr为左臂
    x_flag=-1126;
    arpher8=pi/2;
    arpher9=-pi/2;
    theta1=pi+theta1_norm;
    theta2=-theta1;
end


theta9=0;
theta10=0;
%          theta di  ai-1  ar i-1
T1=D_H(theta1,0,x_flag,pi/2);
T2=D_H(theta2,0,1182,0);

temTeata = atan(20/5250);
theta3 = theta3+temTeata;  
T3=D_H(theta3,2000,0,-pi/2);

a4 =215;theta4 = theta4-temTeata;
T4=D_H(theta4,0,a4,-pi/2);
a5 = 5250+L1;
T5=D_H(theta5,0,a5,0);
a6 = 144;d6 = 248; %theta6 = -theta6;
T6=D_H(theta6,d6,a6,pi/2);
a7=0;d7=900; %d7=898 theta7=-theta7
T7=D_H(theta7,d7,a7,-pi/2);
a8 = 240;d8 = 580;
% T8=D_H(theta8,d8,a8,pi/2);
T8=D_H(theta8,d8,a8,arpher8);
d9=2440+L2;
T9=D_H(theta9,d9,0,arpher9);%(前端点)
TN=D_H(theta9,0,0,arpher9);%(第9个关节点)
%d10=L3,a10=215;
% T10=D_H(theta10,d10,a10,0);%凿岩机钎杆工具坐标

T01=T1
T02=T1*T2
T03=T02*T3
T04=T03*T4
T05=T04*T5
T06=T05*T6
T07=T06*T7
T08=T07*T8
T09=T08*T9
T0N=T08*TN


% T010=T09*T10;
% T08=T1*T2*T3*T4*T5*T6*T7*T8
% T09=T1*T2*T3*T4*T5*T6*T7*T8*T9
% T010=T1*T2*T3*T4*T5*T6*T7*T8*T9*T10

X1 = T01(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y1 = T01(2,4);
Z1 = T01(3,4);
X2 = T02(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y2 = T02(2,4);
Z2 = T02(3,4);
X3 = T03(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y3 = T03(2,4);
Z3 = T03(3,4);
X4 = T04(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y4 = T04(2,4);
Z4 = T04(3,4);
X5 = T05(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y5 = T05(2,4);
Z5 = T05(3,4);
X6 = T06(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y6 = T06(2,4);
Z6 = T06(3,4);
X7 = T07(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y7 = T07(2,4);
Z7 = T07(3,4);
X8 = T08(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y8 = T08(2,4);
Z8 = T08(3,4);
X9 = T09(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y9 = T09(2,4);
Z9 = T09(3,4);
XN = T0N(1,4);%分别提取4X4位置矩阵的X,Y,Z值
YN = T0N(2,4);
ZN = T0N(3,4);
% X10 = T010(1,4);%分别提取4X4位置矩阵的X,Y,Z值
% Y10 = T010(2,4);
% Z10 = T010(3,4);

X = [X1,X2,X3,X4,X5,X6,X7,X8,X9,XN]';
Y = [Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8,Y9,YN]';
Z= [Z1,Z2,Z3,Z4,Z5,Z6,Z7,Z8,Z9,ZN]';
Joint=[X,Y,Z];%关节

end

