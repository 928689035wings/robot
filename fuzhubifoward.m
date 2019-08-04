
function [fuzhubi_foward]=fuzhubifoward(f_theta1,f_theta2,F1)
% f_theta1=pi/2;
% f_theta2=-pi/2;
f_theta3=0;
% F1=2000;
f5 = 5250+F1;

T1=D_H(f_theta1,1083,0,0);
T2=D_H(f_theta2,0,0,-pi/2);
T3=D_H(f_theta3,f5,0,-pi/2);
T12=T1*T2;
T03=T12*T3;
X1 = T1(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y1 = T1(2,4);
Z1 = T1(3,4);
X3 = T03(1,4);%分别提取4X4位置矩阵的X,Y,Z值
Y3 = T03(2,4);
Z3 = T03(3,4);
fuzhubi_foward=[X1,Y1,Z1,X3,Y3,Z3]
