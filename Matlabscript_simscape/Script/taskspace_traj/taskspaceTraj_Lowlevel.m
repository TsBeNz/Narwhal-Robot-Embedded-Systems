%% คำนวณ C จาก Traj. ด้าน High level
% จากนั้น ส่งค่า C มาให้ Low level คำนวณหา qd qv qa
%% Traj. Gen
%function 
[qd,qv,T] = GenTraj(q0,qf,v0,tf) % vf ,ai , af =0 
    t0 = 0;
%     vmax = pi/2;

%     dq = qf - q0;
%     tf = abs(dq/((vmax-v0)/1.9));

    t = linspace(t0,tf,1000*(tf - t0)); % อันนี้เป็น delta t ไปบวกในเวลาก่อนหน้าไปเรื่อยๆ และลูปเอาได้
    T = tf;
    % แต่ละ C มีขนาด 5x1 ดังนั้นอาจจะต้องลูปทำทีละแถวเพื่อหาสำหรับแต่ละ
    % taskspace แต่ละตัว
    ds = q0 - qf;
    tfv0 = tf*v0;

    co0 = q0
    co1 = v0
    co2 = 0;
    co3 =  - (2*(5*ds + 3*(tfv0)))/tf^3;
    co4 = (15*ds + 8*tfv0)/tf^4;
    co5 =  - (3*(2*ds + tfv0))/tf^5;

    qd = co0 + (co1.*t) + (co3.*t.^3) + (co4.*t.^4) + (co5.*t.^5);
    qv = co1 + (3*co3.*t.^2) + (4*co4.*t.^3) + (5*co5.*t.^4);
    qa = (6*co3.*t) + (12*co4.*t.^2) + (20*co5.*t.^3);
%end
%% IPK
% ในแต่ละลูป
% จะได้ Xd1 Xd2 Xd3 Xd4 Xd5 จาก Traj และได้ Xv1 Xv2 Xv3 Xv4 Xv5
% q1 q2 q3 q4 q5 ได้จาก นำ Xd1 Xd2 Xd3 Xd4 Xd5 ไปใส่ไว้ในฟังก์ชัน IPK
%% IVK (Jacobian)
%qv = newJe * Xv
syms X1 X2 X3 X4 X5 real
syms q1 q2 q3 q4 q5 real
syms wx wy wz vx vy vz h1 h2 l1 l2 l3 real
Xv =[X1; X2; X3; X4; X5];
newJe = [                                                                  0,                                                                    -1,                                                -1,                           -1,                    0;
                                                                           0,                                                                     0,                                                 0,                            0,                    1;
- sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) - l3*sin(q2 + q3 + q4)*sin(q1), l3*cos(q2 + q3 + q4)*cos(q1) - cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -cos(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*cos(q1),                    0;
  cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) + l3*sin(q2 + q3 + q4)*cos(q1), l3*cos(q2 + q3 + q4)*sin(q1) - sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -sin(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*sin(q1),                    0;
                                                                           0,                   l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4),            l2*cos(q2 + q3) + l3*sin(q2 + q3 + q4),         l3*sin(q2 + q3 + q4),                    0];
qv= newJe * Xv      
%%
qv =
 
                                                                                                                                                                                                                                      - X2 - X3 - X4
                                                                                                                                                                                                                                                  X5
X4*l3*cos(q2 + q3 + q4)*cos(q1) - X2*(cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)) - l3*cos(q2 + q3 + q4)*cos(q1)) - X3*cos(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)) - X1*(sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) + l3*sin(q2 + q3 + q4)*sin(q1))
X1*(cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) + l3*sin(q2 + q3 + q4)*cos(q1)) - X2*(sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)) - l3*cos(q2 + q3 + q4)*sin(q1)) - X3*sin(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)) + X4*l3*cos(q2 + q3 + q4)*sin(q1)
                                                                                                                    X2*(l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4)) + X3*(l2*cos(q2 + q3) + l3*sin(q2 + q3 + q4)) + X4*l3*sin(q2 + q3 + q4)
 

