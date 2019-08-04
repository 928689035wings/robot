

function [moduan_distance_input]=main_distance_judgement0530(Ld_guanjie1,Ld_guanjie2,Rd_guanjie1,Rd_guanjie2) 
% logical t1 t2;
% left_Y=Ld_guanjie2+K1*(Ld_guanjie1-Ld_guanjie2)/(norm(Ld_guanjie2-Ld_guanjie1));
% right_Y=Rd_guanjie2+K2*(Rd_guanjie1-Rd_guanjie2)/(norm(Rd_guanjie2-Rd_guanjie1));
%�˴�d1 d2 û������Ϊ��λ���� ������ĩ�˵㴦�Ѿ������˴���  �ڼ���֮����������ʾ��ʱ���ǽ�����ģ�Ŵ����˵� ���Դ˴������޸�

%judge_flag��ʼֵ
distance_flag=9;
judge_flag=9;
min_distance_result=9;
dot_flag1=9;
dot_flag2=9;
L1_pengzhuangpoint=[0 0 0];
R1_pengzhuangpoint=[0 0 0];
L0_pengzhuangpoint=[0 0 0];
R0_pengzhuangpoint=[0 0 0];
L_pengzhuangpoint=[0 0 0];
R_pengzhuangpoint=[0 0 0];


p1=Ld_guanjie2;%(��ؽ�ǰ�˵�)
p2=Rd_guanjie2;%(�ҹؽ�ǰ�˵�)
dn12=Ld_guanjie1;
dn21=Rd_guanjie1;
L_norm=norm(Ld_guanjie2-Ld_guanjie1);
R_norm=norm(Rd_guanjie2-Rd_guanjie1);
% dn12=p1+6028*(Ld_guanjie1-Ld_guanjie2)/norm(Ld_guanjie1-Ld_guanjie2);%(��ؽ�ǰ�˵�=p1+6028*��λ����)
% dn21=p2+6028*(Rd_guanjie1-Rd_guanjie2)/norm(Rd_guanjie1-Rd_guanjie2);%(�ҹؽ�ǰ�˵�)

%��λ����
v1=(Ld_guanjie1-Ld_guanjie2)/norm(Ld_guanjie1-Ld_guanjie2);
v2=(Rd_guanjie1-Rd_guanjie2)/norm(Rd_guanjie1-Rd_guanjie2);


a1=dot(v1,v2);
a2=dot(v2,v2);
b1=-dot(v1,v1);
b2=-dot(v1,v2);
A=[a1 b1;a2 b2];
c1=dot((p1-p2),v1);
c2=dot((p1-p2),v2);
C=[c1 c2]';
X=A\C;
judge_t1=X(2);
judge_t2=X(1);

%���ڼ�������X��ǰ������Ū����
dn_left=p1+X(2)*v1;
dn_right=p2+X(1)*v2;
d_lr=dn_left-dn_right;



%��������̾���
% dn_left=p1+X(2)*v1;
% dn_right=p2+X(1)*v2;
% d_lr=dn_left-dn_right;
norm_d_lr=norm(d_lr);
min_distance_1=norm_d_lr;

%���鹫�����Ƿ������۴�ֱ
dot_flag1=dot(d_lr,v1);
dot_flag2=dot(d_lr,v2);
if dot_flag1>-0.2&&dot_flag1<0.2&&dot_flag2>-0.2&&dot_flag2<0.2
    dot_flag=0;
else
    dot_flag=max([dot_flag1,dot_flag2]);
end
% r1=Ld_guanjie2+judge_t1*(Ld_guanjie1-Ld_guanjie2)/norm(Ld_guanjie1-Ld_guanjie2);
% r2=Rd_guanjie2+judge_t2*(Rd_guanjie1-Rd_guanjie2)/norm(Rd_guanjie1-Rd_guanjie2);


    
%  ��С�˵�����ж�   
l1=norm(p1-p2);%(��1����1)
l2=norm(p1-dn21);%����1����2��
l3=norm(dn12-p2);%����2����1��
l4=norm(dn12-dn21);%����2����2��
l=[l1,l2,l3,l4];
min_distance_2=min(l);
%�˵�λ�ñ�־λ��ʾ
 if  min_distance_2==l1
        judge_flag_2=1;
        L1_pengzhuangpoint=p1;
        R1_pengzhuangpoint=p2;
 elseif min_distance_2==l2
        judge_flag_2=2;
        L1_pengzhuangpoint=p1;
        R1_pengzhuangpoint=dn12;
 elseif min_distance_2==l3
        judge_flag_2=3;
        L1_pengzhuangpoint=dn12;
        R1_pengzhuangpoint=p2;
 elseif min_distance_2==l4
        judge_flag_2=4;
        L1_pengzhuangpoint=dn12;
        R1_pengzhuangpoint=dn21;
  end
if (L_norm>judge_t1)&&(judge_t1>=-10) && (R_norm>judge_t2)&&(judge_t2>=-10)%��ʹ��&&��Ϊ������ж�
    judge_flag=0;
    L0_pengzhuangpoint=dn_left;
    R0_pengzhuangpoint=dn_right;
end

%ʵ�ʱ�־λ����̾����������ʾ
if judge_flag==0
    distance_flag=judge_flag;
    min_distance_result=min_distance_1;
    L_pengzhuangpoint=L0_pengzhuangpoint;
    R_pengzhuangpoint=R0_pengzhuangpoint;
else 
    distance_flag=judge_flag_2;
    min_distance_result=min_distance_2;
    L_pengzhuangpoint=L1_pengzhuangpoint;
    R_pengzhuangpoint=R1_pengzhuangpoint;
end

moduan_distance_input=[judge_t1,judge_t2,min_distance_1,min_distance_2,distance_flag,min_distance_result,dot_flag,L_pengzhuangpoint(1),L_pengzhuangpoint(2),L_pengzhuangpoint(3),R_pengzhuangpoint(1),R_pengzhuangpoint(2),R_pengzhuangpoint(3)];
% moduan_distance_input={Ld_guanjie1,Ld_guanjie2,dn12,Rd_guanjie1,Rd_guanjie2,dn21,[judge_t1,judge_t2,min_distance_1,min_distance_2,distance_flag,min_distance_result,dot_flag]};
% moduan_distance_input=[Ld_guanjie1,Ld_guanjie2,Rd_guanjie1,Rd_guanjie2, distance_flag,min_distance_result,judge_t1,judge_t2];
% moduan_distance_input=[ distance_flag,min_distance_result,judge_t1,judge_t2];

end