  clc;
  clear;
% n=10;
% r=-1*rand(10,1)+0.5
% ft1=pi/2*r(1);
% ft2=105*pi/180*r(2);
% ft3=105*pi/180*r(3);
% ft4=pi/2*r(4);
% ft5=288*pi/180*r(5);
% ft6=105*pi/180*r(6);
% fl1=2200*r(7);
% fl2=1200*r(8);
% fl3=4026*r(9);
%生成原始矩阵


r_flag=1;  %1为右臂，-1为左臂
a1=215;
right_L0=770;
right_L1=2000
right_L2=1000
right_theta1_norm=1.2580-acos((3764374-1540*right_L0-right_L0*right_L0)/3776630)  
right_theta1=-right_theta1_norm;  %必须为负数
right_theta2=right_theta1_norm;
right_theta3=pi/2;
right_theta4=0;
right_theta5=0;
right_theta6=-pi/2;
right_theta7=-pi/2;
right_theta8=0;

l_flag=-1;  %1为右臂，-1为左臂
a1=215;
left_L0=770;
left_L1=2000
left_L2=1000
left_theta1_norm=1.2580-acos((3764374-1540*left_L0-left_L0*left_L0)/3776630)  
left_theta1=-left_theta1_norm;  %必须为负数
left_theta2=left_theta1_norm;
left_theta3=pi/2;
left_theta4=0;
left_theta5=0;
left_theta6=-pi/2;
left_theta7=-pi/2;
left_theta8=0;

 f_theta1=pi/2;
 f_theta2=-pi/2;
 F1=5000;

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
  %辅助臂输入数据
 fuzhubi_guanjie=fuzhubifoward(f_theta1,f_theta2,F1)
 f_guanjie1=fuzhubi_guanjie(1,1:3)
 f_guanjie2=fuzhubi_guanjie(1,4:6)
Xo=f_guanjie2(1)
Yo=f_guanjie2(2)
Zo=f_guanjie2(3)
%  worktable_center0=[Xo,Yo+750,Zo+1100]
 worktable_center1=[Xo,Yo+850,Zo+1100]
 worktable_center2=[Xo,Yo+650,Zo+1100]
%Rt=f_theta2
% R_worktable=[cos(Rt) -sin(Rt) 0;sin(Rt) cos(Rt) 0;0 0 1] 
%  worktable_center=(f_guanjie2-f_guanjie2)+R_worktable*[650,Yo+750,Zo+1100]
 
dabi_output=main_distance_judgement0530(left_guanjie4,left_guanjie5,right_guanjie4,right_guanjie5);%两大臂之间防碰撞
 Ldabi_moduan_output=main_distance_judgement0530(left_guanjie9,L_dn,right_guanjie3,right_guanjie5) %L大臂和前端
Rdabi_moduan_output =main_distance_judgement0530(left_guanjie3,left_guanjie5,right_guanjie9,R_dn) %R大臂和前端
moduan_output=main_distance_judgement0530(left_guanjie9,L_dn,right_guanjie9,R_dn);%末端与末端
fu_Rdabi_output=main_distance_judgement0530(right_guanjie4,right_guanjie5,f_guanjie1,f_guanjie2)%辅助臂与右大臂
fu_Ldabi_output=main_distance_judgement0530(left_guanjie4,left_guanjie5,f_guanjie1,f_guanjie2)%辅助臂与左大臂
fu_R_moduan_output=main_distance_judgement0530(right_guanjie9,R_dn,f_guanjie1,f_guanjie2)%辅助臂与右末端
fu_L_moduan_output=main_distance_judgement0530(left_guanjie9,L_dn,f_guanjie1,f_guanjie2)%辅助臂与左末端
%关节
Lguanjie67_worktable_center=main_distance_judgement0530(left_guanjie6,left_guanjie7,worktable_center1,worktable_center2)
Rguanjie67_worktable_center=main_distance_judgement0530(right_guanjie6,right_guanjie7,worktable_center1,worktable_center2)
Lmoduan_worktable_center=main_distance_judgement0530(left_guanjie9,L_dn,worktable_center1,worktable_center2)
Rmoduan_worktable_center=main_distance_judgement0530(right_guanjie9,R_dn,worktable_center1,worktable_center2)

    dabi_dabi_distance=dabi_output(1,6)
    L_p_dabi_judge(1)=dabi_output(1,8);%L_pengzhuangpoint(1)
    L_p_dabi_judge(2)=dabi_output(1,9);%L_pengzhuangpoint(2)
    L_p_dabi_judge(3)=dabi_output(1,10);%L_pengzhuangpoint(3)
    R_p_dabi_judge(1)=dabi_output(1,11);%R_pengzhuangpoint(1)
    R_p_dabi_judge(2)=dabi_output(1,12);%R_pengzhuangpoint(2)
    R_p_dabi_judge(3)=dabi_output(1,13);%R_pengzhuangpoint(3)  
    
    Ldabi_moduan_distance=Ldabi_moduan_output(1,6);%L_pengzhuangpoint(1)
    L_p_Ldabi_moduan_judge(1)=Ldabi_moduan_output(1,8);%L_pengzhuangpoint(1)
    L_p_Ldabi_moduan_judge(2)=Ldabi_moduan_output(1,9);%L_pengzhuangpoint(2)
    L_p_Ldabi_moduan_judge(3)=Ldabi_moduan_output(1,10);%L_pengzhuangpoint(3)
    R_p_Ldabi_moduan_judge(1)=Ldabi_moduan_output(1,11);%R_pengzhuangpoint(1)
    R_p_Ldabi_moduan_judge(2)=Ldabi_moduan_output(1,12);%R_pengzhuangpoint(2)
    R_p_Ldabi_moduan_judge(3)=Ldabi_moduan_output(1,13);%R_pengzhuangpoint(3)
 
    Rdabi_moduan_distance=Rdabi_moduan_output(1,6);%L_pengzhuangpoint(1)
    L_p_Rdabi_moduan_judge(1)=Rdabi_moduan_output(1,8);%L_pengzhuangpoint(1)
    L_p_Rdabi_moduan_judge(2)=Rdabi_moduan_output(1,9);%L_pengzhuangpoint(2)
    L_p_Rdabi_moduan_judge(3)=Rdabi_moduan_output(1,10);%L_pengzhuangpoint(3)
    R_p_Rdabi_moduan_judge(1)=Rdabi_moduan_output(1,11);%R_pengzhuangpoint(1)
    R_p_Rdabi_moduan_judge(2)=Rdabi_moduan_output(1,12);%R_pengzhuangpoint(2)
    R_p_Rdabi_moduan_judge(3)=Rdabi_moduan_output(1,13);%R_pengzhuangpoint(3)
  
    moduan_moduan_distance=moduan_output(1,6);%L_pengzhuangpoint(1)
    L_p_moduan_judge(1)=moduan_output(1,8);%L_pengzhuangpoint(1)
    L_p_moduan_judge(2)=moduan_output(1,9);%L_pengzhuangpoint(2)
    L_p_moduan_judge(3)=moduan_output(1,10);%L_pengzhuangpoint(3)
    R_p_moduan_judge(1)=moduan_output(1,11);%R_pengzhuangpoint(1)
    R_p_moduan_judge(2)=moduan_output(1,12);%R_pengzhuangpoint(2)
    R_p_moduan_judge(3)=moduan_output(1,13);%R_pengzhuangpoint(3)
  
    fu_Rdabi_distance=fu_Rdabi_output(1,6);%L_pengzhuangpoint(1)
    L_p_fu_Rdabi_judge(1)=fu_Rdabi_output(1,8);%L_pengzhuangpoint(1)
    L_p_fu_Rdabi_judge(2)=fu_Rdabi_output(1,9);%L_pengzhuangpoint(2)
    L_p_fu_Rdabi_judge(3)=fu_Rdabi_output(1,10);%L_pengzhuangpoint(3)
    R_p_fu_Rdabi_judge(1)=fu_Rdabi_output(1,11);%R_pengzhuangpoint(1)
    R_p_fu_Rdabi_judge(2)=fu_Rdabi_output(1,12);%R_pengzhuangpoint(2)
    R_p_fu_Rdabi_judge(3)=fu_Rdabi_output(1,13);%R_pengzhuangpoint(3)
    
     fu_Ldabi_distance=fu_Ldabi_output(1,6);%L_pengzhuangpoint(1)
    L_p_fu_Ldabi_judge(1)=fu_Ldabi_output(1,8);%L_pengzhuangpoint(1)
    L_p_fu_Ldabi_judge(2)=fu_Ldabi_output(1,9);%L_pengzhuangpoint(2)
    L_p_fu_Ldabi_judge(3)=fu_Ldabi_output(1,10);%L_pengzhuangpoint(3)
    R_p_fu_Ldabi_judge(1)=fu_Ldabi_output(1,11);%R_pengzhuangpoint(1)
    R_p_fu_Ldabi_judge(2)=fu_Ldabi_output(1,12);%R_pengzhuangpoint(2)
    R_p_fu_Ldabi_judge(3)=fu_Ldabi_output(1,13);%R_pengzhuangpoint(3)
 
     fu_R_moduan_distance=fu_R_moduan_output(1,6);%L_pengzhuangpoint(1)
    L_p_fu_R_moduan_judge(1)=fu_R_moduan_output(1,8);%L_pengzhuangpoint(1)
    L_p_fu_R_moduan_judge(2)=fu_R_moduan_output(1,9);%L_pengzhuangpoint(2)
    L_p_fu_R_moduan_judge(3)=fu_R_moduan_output(1,10);%L_pengzhuangpoint(3)
    R_p_fu_R_moduan_judge(1)=fu_R_moduan_output(1,11);%R_pengzhuangpoint(1)
    R_p_fu_R_moduan_judge(2)=fu_R_moduan_output(1,12);%R_pengzhuangpoint(2)
    R_p_fu_R_moduan_judge(3)=fu_R_moduan_output(1,13);%R_pengzhuangpoint(3)
    
     fu_L_moduan_distance=fu_L_moduan_output(1,6);%L_pengzhuangpoint(1)
    L_p_fu_L_moduan_judge(1)=fu_L_moduan_output(1,8);%L_pengzhuangpoint(1)
    L_p_fu_L_moduan_judge(2)=fu_L_moduan_output(1,9);%L_pengzhuangpoint(2)
    L_p_fu_L_moduan_judge(3)=fu_L_moduan_output(1,10);%L_pengzhuangpoint(3)
    R_p_fu_L_moduan_judge(1)=fu_L_moduan_output(1,11);%R_pengzhuangpoint(1)
    R_p_fu_L_moduan_judge(2)=fu_L_moduan_output(1,12);%R_pengzhuangpoint(2)
    R_p_fu_L_moduan_judge(3)=fu_L_moduan_output(1,13);%R_pengzhuangpoint(3)
    

    Lguanjie67_worktable_center_distance=Lguanjie67_worktable_center(1,6);
    Lguanjie67_judge(1)=Lguanjie67_worktable_center(1,8);
    Lguanjie67_judge(2)=Lguanjie67_worktable_center(1,9);
    Lguanjie67_judge(3)=Lguanjie67_worktable_center(1,10);
    Lworktable_center_judge(1)=Lguanjie67_worktable_center(1,11);
    Lworktable_center_judge(2)=Lguanjie67_worktable_center(1,12);
    Lworktable_center_judge(3)=Lguanjie67_worktable_center(1,13);
    
    
      Rguanjie67_worktable_center_distance=Rguanjie67_worktable_center(1,6);
    Rguanjie67_judge(1)=Rguanjie67_worktable_center(1,8);
    Rguanjie67_judge(2)=Rguanjie67_worktable_center(1,9);
    Rguanjie67_judge(3)=Rguanjie67_worktable_center(1,10);
    Rworktable_center_judge(1)=Rguanjie67_worktable_center(1,11);
    Rworktable_center_judge(2)=Rguanjie67_worktable_center(1,12);
    Rworktable_center_judge(3)=Rguanjie67_worktable_center(1,13);  
    
          Lmoduan_worktable_center_distance=Lmoduan_worktable_center(1,6);
    Lmoduan_worktable_center_judge(1)=Lmoduan_worktable_center(1,8);
    Lmoduan_worktable_center_judge(2)=Lmoduan_worktable_center(1,9);
    Lmoduan_worktable_center_judge(3)=Lmoduan_worktable_center(1,10);
    worktable_center_Lmoduan_judge(1)=Lmoduan_worktable_center(1,11);
    worktable_center_Lmoduan_judge(2)=Lmoduan_worktable_center(1,12);
    worktable_center_Lmoduan_judge(3)=Lmoduan_worktable_center(1,13); 

           Rmoduan_worktable_center_distance=Rmoduan_worktable_center(1,6);
    Rmoduan_worktable_center_judge(1)=Rmoduan_worktable_center(1,8);
    Rmoduan_worktable_center_judge(2)=Rmoduan_worktable_center(1,9);
    Rmoduan_worktable_center_judge(3)=Rmoduan_worktable_center(1,10);
    worktable_center_Rmoduan_judge(1)=Rmoduan_worktable_center(1,11);
    worktable_center_Rmoduan_judge(2)=Rmoduan_worktable_center(1,12);
    worktable_center_Rmoduan_judge(3)=Rmoduan_worktable_center(1,13); 
    %  展现关节点 可视化
figure 
 W=[-15 +15  -15 +15 -15 +15]*800;
   plot3([left_guanjie1(1) left_guanjie2(1) left_guanjie3(1) left_guanjie4(1) left_guanjie5(1) left_guanjie6(1) left_guanjie7(1) left_guanjie8(1) left_guanjie9(1) left_guanjie_N(1)],...
    [left_guanjie1(2) left_guanjie2(2) left_guanjie3(2) left_guanjie4(2) left_guanjie5(2) left_guanjie6(2) left_guanjie7(2) left_guanjie8(2) left_guanjie9(2) left_guanjie_N(2)],...
    [left_guanjie1(3) left_guanjie2(3) left_guanjie3(3) left_guanjie4(3) left_guanjie5(3) left_guanjie6(3) left_guanjie7(3) left_guanjie8(3) left_guanjie9(3)  left_guanjie_N(3)],'g',...
    [L_dn(1) left_guanjie9(1)], [L_dn(2) left_guanjie9(2)], [L_dn(3) left_guanjie9(3)],'b'),grid on%工作装置
hold on
    plot3([right_guanjie1(1) right_guanjie2(1) right_guanjie3(1) right_guanjie4(1) right_guanjie5(1) right_guanjie6(1) right_guanjie7(1) right_guanjie8(1) right_guanjie9(1) right_guanjie_N(1)],...
    [right_guanjie1(2) right_guanjie2(2) right_guanjie3(2) right_guanjie4(2) right_guanjie5(2) right_guanjie6(2) right_guanjie7(2) right_guanjie8(2) right_guanjie9(2) right_guanjie_N(2)],...
    [right_guanjie1(3) right_guanjie2(3) right_guanjie3(3) right_guanjie4(3) right_guanjie5(3) right_guanjie6(3) right_guanjie7(3) right_guanjie8(3) right_guanjie9(3)  right_guanjie_N(3)],'g',...
    [R_dn(1) right_guanjie9(1)], [R_dn(2) right_guanjie9(2)], [R_dn(3) right_guanjie9(3)],'b',...%工作装置
    [f_guanjie1(1) f_guanjie2(1)],...
    [f_guanjie1(2) f_guanjie2(2)],...
    [f_guanjie1(3) f_guanjie2(3)],'g'),grid on%判断线
hold on 
%工作台显示
o=[Xo-650,Yo,Zo];
L=1300;W=1500;H=2200;
% T_worktable=[cos(Rt) -sin(Rt) 0 0;sin(Rt) cos(Rt) 0 0;0 0 1 1] 
x=[o(1),o(1)+L,o(1)+L,o(1),o(1);o(1),o(1)+L,o(1)+L,o(1),o(1)];
y=[o(2),o(2),o(2)+W,o(2)+W,o(2);o(2),o(2),o(2)+W,o(2)+W,o(2)];
z=[o(3),o(3),o(3),o(3),o(3);o(3)+H,o(3)+H,o(3)+H,o(3)+H,o(3)+H];
mesh(x,y,z)
colormap(gray(1));
xlabel('x');ylabel('y');zlabel('z')



% ;x=[0 20 20 0 0 0 0 0 0 0 0 20 20 20 20 20 20 20 0];y=[0 0 0 0 0 0 20 20 0 20 20 20 20 0 0 20 20 20 20];z=[0 0 40 40 0 40 40 0 0 0 40 40 0 0 40 40 0 0 0];plot3(x,y,z,'r');
    
%       hold on
safedistance=500
   if  dabi_dabi_distance <=300+safedistance
     plot3([L_p_dabi_judge(1) R_p_dabi_judge(1)],[L_p_dabi_judge(2) R_p_dabi_judge(2)],[L_p_dabi_judge(3) R_p_dabi_judge(3)],'r'),grid on
  hold on
   end
   if  Ldabi_moduan_distance <=300+safedistance
    plot3([L_p_Ldabi_moduan_judge(1) R_p_Ldabi_moduan_judge(1)],[L_p_Ldabi_moduan_judge(2) R_p_Ldabi_moduan_judge(2)],[L_p_Ldabi_moduan_judge(3) R_p_Ldabi_moduan_judge(3)],'r')
   hold on
   end
if Rdabi_moduan_distance  <=300+safedistance
    plot3([L_p_Rdabi_moduan_judge(1) R_p_Rdabi_moduan_judge(1)],[L_p_Rdabi_moduan_judge(2) R_p_Rdabi_moduan_judge(2)],[L_p_Rdabi_moduan_judge(3) R_p_Rdabi_moduan_judge(3)],'r')
hold on
end
if moduan_moduan_distance <=300+safedistance
    plot3([L_p_moduan_judge(1) R_p_moduan_judge(1)],[L_p_moduan_judge(2) R_p_moduan_judge(2)],[L_p_moduan_judge(3) R_p_moduan_judge(3)],'r')
hold on
end
if fu_Rdabi_distance <=300+safedistance
    plot3([L_p_fu_Rdabi_judge(1) R_p_fu_Rdabi_judge(1)],[L_p_fu_Rdabi_judge(2) R_p_fu_Rdabi_judge(2)],[L_p_fu_Rdabi_judge(3) R_p_fu_Rdabi_judge(3)],'r')
hold on
end
if fu_Ldabi_distance<=300+safedistance
    plot3([L_p_fu_Ldabi_judge(1) R_p_fu_Ldabi_judge(1)],[L_p_fu_Ldabi_judge(2) R_p_fu_Ldabi_judge(2)],[L_p_fu_Ldabi_judge(3) R_p_fu_Ldabi_judge(3)],'r')
hold on
end
if fu_R_moduan_distance <=300+safedistance
    plot3([L_p_fu_R_moduan_judge(1) R_p_fu_R_moduan_judge(1)],[L_p_fu_R_moduan_judge(2) R_p_fu_R_moduan_judge(2)],[L_p_fu_R_moduan_judge(3) R_p_fu_R_moduan_judge(3)],'r')
hold on
end
if fu_L_moduan_distance<=300+safedistance
    plot3([L_p_fu_L_moduan_judge(1) R_p_fu_L_moduan_judge(1)],[L_p_fu_L_moduan_judge(2) R_p_fu_L_moduan_judge(2)],[L_p_fu_L_moduan_judge(3) R_p_fu_L_moduan_judge(3)],'r'),grid on%判断线
end
hold on

if Rguanjie67_worktable_center_distance<=1000+safedistance
        plot3([Rguanjie67_judge(1) Rworktable_center_judge(1)],[Rguanjie67_judge(2) Rworktable_center_judge(2)],[Rguanjie67_judge(3) Rworktable_center_judge(3)],'r'),grid on%判断线
end
hold on
if Lguanjie67_worktable_center_distance<=5000+safedistance
        plot3([Lguanjie67_judge(1) Lworktable_center_judge(1)],[Lguanjie67_judge(2) Lworktable_center_judge(2)],[Lguanjie67_judge(3) Lworktable_center_judge(3)],'r'),grid on%判断线
end
hold on
if Lmoduan_worktable_center_distance<=1000+safedistance
        plot3([Lmoduan_worktable_center_judge(1) worktable_center_Lmoduan_judge(1)],[Lmoduan_worktable_center_judge(2) worktable_center_Lmoduan_judge(2)],[Lmoduan_worktable_center_judge(3) worktable_center_Lmoduan_judge(3)],'r'),grid on%判断线
end
hold on
if Rmoduan_worktable_center_distance<=1000+safedistance
        plot3([Rmoduan_worktable_center_judge(1) worktable_center_Rmoduan_judge(1)],[Rmoduan_worktable_center_judge(2) worktable_center_Rmoduan_judge(2)],[Rmoduan_worktable_center_judge(3) worktable_center_Rmoduan_judge(3)],'r'),grid on%判断线
end
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
% 




