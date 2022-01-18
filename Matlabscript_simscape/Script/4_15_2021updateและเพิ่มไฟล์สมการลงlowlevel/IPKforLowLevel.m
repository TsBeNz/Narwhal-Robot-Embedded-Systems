syms h1 h2 l1 l2 l3 x y z pitch yaw
%% Input
PositionInput =[x y z pitch yaw];
rotationbar = [0 pitch yaw];
% DH =[0   0   h1   0;
%     l1 pi/2  0  pi/2;
%     h2  0    0  -pi/2;
%     l2  0    0    0;
%     0  pi/2  0    0];
%% first function IKnawhale2
[qbar,shout] = IKnawhale2(P,rbar,gammabar,h1,h2,l1,l2,l3,DH);
    flag =0
%% eul2rotm
%syms pitch yaw real
%R0e = eul2rotm([0 pitch yaw],'ZYZ') ใช้ใน controller ไม่ได้
syms z1 pitch yaw
c =@(x) cos(x);
s =@(x) sin(x);
Rotz =@(a) [c(a) -s(a) 0;
            s(a)  c(a) 0;
            0      0   1];
Roty =@(a) [c(a)   0  s(a);
            0      1   0 ;
           s(a)   0   c(a)];
Rotx =@(a) [1  c(a) -s(a);
            0  s(a)  c(a);
            0    0     0];       
%R0e = eul2rotm([0 -pi pi],'ZYZ')
%R0eME = Rotz(0)*Roty(pitch)*Rotz(yaw)
% จะได้ rotm ที่แปลงแล้วมา
R0e = [cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch)]
      [           sin(yaw),             cos(yaw),          0]
      [cos(yaw)*sin(pitch), -sin(pitch)*sin(yaw), cos(pitch)]
%% ทำการหา position เฟรม 4 เทียบเฟรม 0
syms x y z q1 q2 q3 q4 q5 l3 real
% H3 =
% [floor(cos(q2 + q3)*cos(q1)), floor(-sin(q2 + q3)*cos(q1)),  floor(sin(q1)), floor(cos(q1)*(l1 - h2*sin(q2)))]
% [floor(cos(q2 + q3)*sin(q1)), floor(-sin(q2 + q3)*sin(q1)), floor(-cos(q1)), floor(sin(q1)*(l1 - h2*sin(q2)))]
% [        floor(sin(q2 + q3)),          floor(cos(q2 + q3)),               0,           floor(h1 + h2*cos(q2))]
% [                          0,                            0,               0,                                1]

% Hoe =
% [sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5),   cos(q5)*sin(q1) -
% cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4))]
% [cos(q2 + q3 + q4)*cos(q5)*sin(q1) - cos(q1)*sin(q5), - cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4))]
% [                          sin(q2 + q3 + q4)*cos(q5),                            -sin(q2 + q3 + q4)*sin(q5),        -cos(q2 + q3 + q4),           h1 + l2*sin(q2 + q3) + h2*cos(q2) - l3*cos(q2 + q3 + q4)]
% [                                                  0,                                                     0,                         0,                                                                  1]

Rotoe = [sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5),   cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1);
         cos(q2 + q3 + q4)*cos(q5)*sin(q1) - cos(q1)*sin(q5), - cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1);
                                   sin(q2 + q3 + q4)*cos(q5), -sin(q2 + q3 + q4)*sin(q5),        -cos(q2 + q3 + q4)];
%P004 = [x;y;z]-(Rotoe*[0;0;l3])
P004 = [ x - l3*sin(q2 + q3 + q4)*cos(q1);
         y - l3*sin(q2 + q3 + q4)*sin(q1);
          z + l3*cos(q2 + q3 + q4)]
%% second function IPK2
[q1,q2,q3,flag] = IPK2(P,P004,gammabar,h1,h2,l1,l2,l3,flag);
    q1 = atan2(gammabar(1)*P(2),gammabar(1)*P(1));
%% third function RRIK2
[q2,q3,flag]= RRIK2(P004,gammabar,h1,h2,l1,l2,flag)
    x24 = (x - l3*sin(q2 + q3 + q4)*cos(q1)) - l1;
    z24 = (z + l3*cos(q2 + q3 + q4)) - h1;
    if (sqrt(x24^2+z24^2)<= (l2+h2)) && (sqrt(x24^2+z24^2)>=(l2-h2))% เช็ค
        s3 = (x24^2+z24^2-h2^2-l2^2)/(2*l2*h2);
        c3 = gammabar(2)*sqrt(1-s3^2);
        q2 = atan2(-(h2+l2*s3)*x24 + (l2*c3*z24) , (l2*c3*x24)+(l2*s3+h2));
        q3 = atan2(s3,c3);
    else 
        % ถ้า เงื่อนไขไม่ตรงแสดงว่า input มาจะทำให้หุ่นผิดท่า
        % ต้องไม่คำนวณ
        flag = 1
    end
%% กลับมาใน IKnawhale2
    if flag == 0
        % หา R03 แล้วหา R3e จากนั้น เรียก IOK2 จะได้ q4 q5
        % จะได้ครบ ทุก q แล้ว
    else
        % แจ้งว่า input ไม่ถูกต้อง
    end
%% 
syms q1 q2 q3 q4 q5 pitch yaw real
R03 =[cos(q2 + q3)*cos(q1),        -sin(q2 + q3)*cos(q1),        sin(q1);
      cos(q2 + q3)*sin(q1),        -sin(q2 + q3)*sin(q1),         -cos(q1);
              sin(q2 + q3),                 cos(q2 + q3),             0 ];
R0e =[cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch);
                 sin(yaw),             cos(yaw),          0;
      cos(yaw)*sin(pitch), -sin(pitch)*sin(yaw), cos(pitch)];
%[q4,q5]= IOK2((R03.')*R0e,gammabar);
% R3e = (R03.')*R0e
R3e = [sin(q2 + q3)*cos(yaw)*sin(pitch) + cos(q2 + q3)*sin(q1)*sin(yaw) + cos(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw), cos(q2 + q3)*cos(yaw)*sin(q1) - sin(q2 + q3)*sin(pitch)*sin(yaw) - cos(q2 + q3)*cos(pitch)*cos(q1)*sin(yaw), sin(q2 + q3)*cos(pitch) + cos(q2 + q3)*cos(q1)*sin(pitch);
       cos(q2 + q3)*cos(yaw)*sin(pitch) - sin(q2 + q3)*sin(q1)*sin(yaw) - sin(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw), sin(q2 + q3)*cos(pitch)*cos(q1)*sin(yaw) - sin(q2 + q3)*cos(yaw)*sin(q1) - cos(q2 + q3)*sin(pitch)*sin(yaw), cos(q2 + q3)*cos(pitch) - sin(q2 + q3)*cos(q1)*sin(pitch);
                                                             cos(pitch)*cos(yaw)*sin(q1) - cos(q1)*sin(yaw),                                                            - cos(q1)*cos(yaw) - cos(pitch)*sin(q1)*sin(yaw),                                        sin(pitch)*sin(q1)];
%% forth function IOK2
[q4,q5] = IOK2(R3e,gamma)
    r11 = sin(q2 + q3)*cos(yaw)*sin(pitch) + cos(q2 + q3)*sin(q1)*sin(yaw) + cos(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw);
    r21 = cos(q2 + q3)*cos(yaw)*sin(pitch) - sin(q2 + q3)*sin(q1)*sin(yaw) - sin(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw);
    r31 =  cos(pitch)*cos(yaw)*sin(q1) - cos(q1)*sin(yaw);
    c5 = gamma(3)*sqrt(r11^2+r21^2);
    q5 = atan2(r31,c5);
    q4 = atan2(r21*gamma(3),r11*gamma(3));


