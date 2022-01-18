syms q1 q2 q3 q4 q5 wx wy wz vx vy vz h1 h2 l1 l2 l3 real
c =@(x) cos(x);
s =@(x) sin(x);
% % link length Update
% h1= 275.99; % lasted
% h2= 380;
% l1= 20.01;
% l2= 380;
% l3= 235;
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
q=[q1;q2;q3;q4;q5];
rho = [1;1;1;1;1];
Hne=[1 0 0 0 ;
     0 1 0 0; 
     0 0 1 l3;
     0 0 0 1];
 xi = [wx; wy; wz; vx; vy; vz];
 %%  หา Jacobian ก่อน เป็น Jacobian ที่คูณ Jrpy แล้วเนื่องจากมี input เป็น rpy ด้วย
H = forwardKinematics(q,rho,DH,Hne);
[J,Je] = manipulatorJacobian(q,rho,DH,Hne)
Jrpy = [0  -s(q1) c(q1)*s(q2+q3+q4);
        0   c(q1) s(q1)*s(q2+q3+q4);
        1        0     c(q2+q3+q4)];
Jerpy = Jrpy \ Je(1:3,:);
newJe = [Jerpy ; Je(4:6,:)]
ReducedJe = simplify(newJe(2:end,:));
Reducedxi = xi(2:end);
DetRJe = simplify(det(ReducedJe));
%% ทำ IVK หา ความเร็วของ joints
q_dot = simplify(ReducedJe \ Reducedxi);
%% รูปที่ย่อแล้ว
% q_dot = [         (vy*cos(q1) - vx*sin(q1))/(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4));
%             -(l3*wy*cos(q4) + vz*sin(q3+q2)+ vx*cos(q1)*cos(q2+q3)+vy*sin(q1)*cos(q2+q3)) /(h2*cos(q3));
%             (h2*vz*cos(q2) - h2*vy*sin(q1)*sin(q2)+ l2*l3*wy*cos(q4) - h2*vx*cos(q1)*sin(q2)+ l2*vz*sin(q3+q2)+ h2*l3*wy*sin(q4+q3)+ l2*vx*cos(q1)*cos(q2+q3)+ l2*vy*sin(q1)*cos(q2+q3))/(h2*l2*cos(q3)) ;
%             -(vz*cos(q2) + l2*wy*cos(q3) - vx*cos(q1)*sin(q2) - vy*sin(q1)*sin(q2) + l3*wy*sin(q4+q3))/(l2*cos(q3));
%                                                      wz];
% newJe =
%  
% [                                                                          1,                                                                     0,                                                 0,                            0, -2*cos(q2 + q3 + q4);
%                                                                            0,                                                                    -1,                                                -1,                           -1,                    0;
%                                                                            0,                                                                     0,                                                 0,                            0,                    1;
% - sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) - l3*sin(q2 + q3 + q4)*sin(q1), l3*cos(q2 + q3 + q4)*cos(q1) - cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -cos(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*cos(q1),                    0;
%   cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) + l3*sin(q2 + q3 + q4)*cos(q1), l3*cos(q2 + q3 + q4)*sin(q1) - sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -sin(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*sin(q1),                    0;
%                                                                            0,                   l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4),            l2*cos(q2 + q3) + l3*sin(q2 + q3 + q4),         l3*sin(q2 + q3 + q4),                    0]
%  

%% ทดสอบความถูกต้อง
%ทำ IVK ได้ q_dot เอา q_dot ไปแทนหา fvk  clear workspace
%ก่อนรันเฉพาะส่วนนี้
% % link length Update
h1= 275.99; % lasted
h2= 380;
l1= 20.01;
l2= 380;
l3= 235;
q1 = 0;
q2 = pi/6;
q3 = -pi/6;
q4 = 0;
q5 = 0;
wx = 0;
wy = 0.2;
wz = 0.3;
vx = 50;
vy = 25;
vz = 25;
%input xi ,q  
q_dot = [         (vy*cos(q1) - vx*sin(q1))/(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4));
            -(l3*wy*cos(q4) + vz*sin(q3+q2)+ vx*cos(q1)*cos(q2+q3)+vy*sin(q1)*cos(q2+q3)) /(h2*cos(q3));
            (h2*vz*cos(q2) - h2*vy*sin(q1)*sin(q2)+ l2*l3*wy*cos(q4) - h2*vx*cos(q1)*sin(q2)+ l2*vz*sin(q3+q2)+ h2*l3*wy*sin(q4+q3)+ l2*vx*cos(q1)*cos(q2+q3)+ l2*vy*sin(q1)*cos(q2+q3))/(h2*l2*cos(q3)) ;
            -(vz*cos(q2) + l2*wy*cos(q3) - vx*cos(q1)*sin(q2) - vy*sin(q1)*sin(q2) + l3*wy*sin(q4+q3))/(l2*cos(q3));
                                                     wz];
% q_dot มี q1 q2 q3 q4 q5
%%%%%%%% fvk
newJe = [                                                                  0,                                                                    -1,                                                -1,                           -1,                    0;
                                                                           0,                                                                     0,                                                 0,                            0,                    1;
- sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) - l3*sin(q2 + q3 + q4)*sin(q1), l3*cos(q2 + q3 + q4)*cos(q1) - cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -cos(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*cos(q1),                    0;
  cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) + l3*sin(q2 + q3 + q4)*cos(q1), l3*cos(q2 + q3 + q4)*sin(q1) - sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -sin(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*sin(q1),                    0;
                                                                           0,                   l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4),            l2*cos(q2 + q3) + l3*sin(q2 + q3 + q4),         l3*sin(q2 + q3 + q4),                    0];
                                                                       
xi = newJe * q_dot       
%% %input xi ,q  (ลดรูป 1 จัดรูป)
v_q1 = (vy*cos(q1) - vx*sin(q1))/(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4))
v_q2 = -(l3*wy*cos(q4) + vz*sin(q3+q2)+ vx*cos(q1)*cos(q2+q3)+vy*sin(q1)*cos(q2+q3)) /(h2*cos(q3))
v_q3 = (h2*vz*cos(q2) - h2*vy*sin(q1)*sin(q2)+ l2*l3*wy*cos(q4) - h2*vx*cos(q1)*sin(q2)+ l2*vz*sin(q3+q2)+ h2*l3*wy*sin(q4+q3)+ l2*vx*cos(q1)*cos(q2+q3)+ l2*vy*sin(q1)*cos(q2+q3))/(h2*l2*cos(q3))
v_q4 = -(vz*cos(q2) + l2*wy*cos(q3) - vx*cos(q1)*sin(q2) - vy*sin(q1)*sin(q2) + l3*wy*sin(q4+q3))/(l2*cos(q3))
v_q5 = wz
% q_dot มี q1 q2 q3 q4 q5
%%%%%%%% fvk

%% %input xi ,q  (ลดรูป 2 ตัวซ้ำ เเทนค่า)

pitch = q2 + q3 + q4;
s1 = sin(q1);
c1 = cos(q1);
s2 = sin(q2);
c2 = cos(q2);
c3 = cos(q3);
c4 = cos(q4);
s23 = sin(q3+q2);
c23 = cos(q2+q3);
s34 = sin(q4+q3);

v_q1 = (vy*c1 - vx*s1)/(20.01 + 380*c23 - 380*s2 + 235*sin(pitch))
v_q2 = -(235*wy*c4 + vz*s23+ vx*c1*c23+vy*s1*c23) /(380*c3)
v_q3 = (380*vz*c2 - 380*vy*s1*s2+ 89300*wy*c4 - 380*vx*c1*s2+ 380*vz*s23+ 89300*wy*s34+ 380*vx*c1*c23+ 380*vy*s1*c23)/(144400*c3)
v_q4 = -(vz*c2 + 380*wy*c3 - vx*c1*s2 - vy*s1*s2 + 235*wy*s34)/(380*c3)
v_q5 = wz
% q_dot มี q1 q2 q3 q4 q5
%%%%%%%% fvk
xi
xi2 = newJe * [v_q1;v_q2;v_q3;v_q4;v_q5]