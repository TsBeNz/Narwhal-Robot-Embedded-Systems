%% Input
% link length Update
h1= 275.99; % lasted
h2= 380;
l1= 20.01;
l2= 380;
l3= 235;
%Taskspace
x =(l2+l1);
y =0;
z =(h1+h2-l3);
pitch =-pi;
yaw =pi;
% c =@(x) cos(x);
% s =@(x) sin(x);
% Rotz =@(a) [c(a) -s(a) 0;
%             s(a)  c(a) 0;
%             0      0   1];
% Roty =@(a) [c(a)   0  s(a);
%             0      1   0 ;
%            s(a)   0   c(a)];
% Rotx =@(a) [1  c(a) -s(a);
%             0  s(a)  c(a);
%             0    0     0];    
% R0eME = Rotz(0)*Roty(pitch)*Rotz(yaw)
%% 
% function IKnarwhal(x,y,z,pitch,yaw) %parameters น่าจะอยู่ใน code setup
    % หา R0e = เอา pitch yaw มาแปลง
    % หา P004 
    % เรียก function IPK ได้ q1 q2 q3
        % จะได้ q1 เลย
        q1 = atan2(y,x);
        % เรียก function RRIK ซึ่งจะทำให้ได้ q2 q3
            % คำนวณหา P023
             x24 = (x - l3*sin(pitch)) - l1;
             z24 = (z - l3*cos(pitch)) - h1;
            % จะมีการ เช็ค เงื่อนไข input หนึ่งครั้ง
%             if (sqrt(x24^2+z24^2)<= (l2+h2)) && (sqrt(x24^2+z24^2)>=(l2-h2))
                s3 = (x24^2+z24^2-h2^2-l2^2)/(2*l2*h2);
                c3 = sqrt(1-s3^2);
                q2 = atan2(-(h2+l2*s3)*x24 + (l2*c3*z24) , (l2*c3*x24)+(l2*s3+h2));
                q3 = atan2(s3,c3);
%             end
    % ได้ q1 q2 q3 
    % ทำการ R03
    % ทำการหา R3e = (R03.')*R0e
    % จะเรียก function IOK
        r11 = sin(q2 + q3)*cos(yaw)*sin(pitch) + cos(q2 + q3)*sin(q1)*sin(yaw) + cos(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw);
        r21 = cos(q2 + q3)*cos(yaw)*sin(pitch) - sin(q2 + q3)*sin(q1)*sin(yaw) - sin(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw);
        r31 = cos(pitch)*cos(yaw)*sin(q1) - cos(q1)*sin(yaw);
        c5 = sqrt(r11^2+r21^2);
        q5 = atan2(r31,c5);
        q4 = atan2(r21,r11);

    % จะได้ q4 q5
% end
%% 
q1 = atan2(y,x);
x24 = (x - l3*sin(pitch)) - l1;
z24 = (z - l3*cos(pitch)) - h1;
  
if (sqrt(x24^2+z24^2)<= (l2+h2)) && (sqrt(x24^2+z24^2)>=(l2-h2))
    s3 = (x24^2+z24^2-h2^2-l2^2)/(2*l2*h2);
    c3 = sqrt(1-s3^2);
    q2 = atan2(-(h2+l2*s3)*x24 + (l2*c3*z24) , (l2*c3*x24)+(l2*s3+h2))
    q3 = atan2(s3,c3);
    r11 = sin(q2 + q3)*cos(yaw)*sin(pitch) + cos(q2 + q3)*sin(q1)*sin(yaw) + cos(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw);
    r21 = cos(q2 + q3)*cos(yaw)*sin(pitch) - sin(q2 + q3)*sin(q1)*sin(yaw) - sin(q2 + q3)*cos(pitch)*cos(q1)*cos(yaw);
    r31 = cos(pitch)*cos(yaw)*sin(q1) - cos(q1)*sin(yaw);
    c5 = sqrt(r11^2+r21^2);
    q5 = atan2(r31,c5);
    q4 = atan2(r21,r11);
    q = [q1;q2;q3;q4;q5]
else
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = 0
    q5 = 0
end
%% for low LV ลดรูปพจน์ซ้ำ
q1 = atan2(y,x);

%define part 1
s_pitch = sin(pitch);
c_pitch = cos(pitch);

x24 = (x - l3*s_pitch) - l1;
z24 = (z - l3*c_pitch) - h1;

%define part 2
x24_z24_pow2 = (x24^2+z24^2); 

if (sqrt(x24_z24_pow2)<= (l2+h2)) && (sqrt(x24_z24_pow2)>=(l2-h2)) 
    s3 = (x24_z24_pow2-h2^2-l2^2)/(2*l2*h2)
    c3 = sqrt(1-s3^2);
    q2 = atan2(-(h2+l2*s3)*x24 + (l2*c3*z24) , (l2*c3*x24)+(l2*s3+h2))
    q3 = atan2(s3,c3);
    
    %define part 3    
    s1 = sin(q1);
    c1 = cos(q1);
    s23 = sin(q2 + q3);
    c23 = cos(q2 + q3);
    s_yaw = sin(yaw);
    c_yaw = cos(yaw);
    
    r11 = s23*c_yaw*s_pitch + c23*s1*s_yaw + c23*c_pitch*c1*c_yaw;
    r21 = c23*c_yaw*s_pitch - s23*s1*s_yaw - s23*c_pitch*c1*c_yaw;
    r31 = c_pitch*c_yaw*s1 - c1*s_yaw;
    c5 = sqrt(r11^2+r21^2);
    q5 = atan2(r31,c5);
    q4 = atan2(r21,r11);
    q = [q1;q2;q3;q4;q5]
else
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = 0
    q5 = 0
end
%% %% for low LV ลดรูปพจน์ซ้ำเเละเเทนค่าเเล้ว
q1 = atan2(y,x);

%define part 1
s_pitch = sin(pitch);
c_pitch = cos(pitch);

x24 = (x - 235*s_pitch) - 20.01;
z24 = (z - 235*c_pitch) - 275.99;

%define part 2
x24_z24_pow2 = ((x24*x24)+(z24*z24));

if (sqrt(x24_z24_pow2)<= (760))
    s3 = (x24_z24_pow2-288800)/(288800);
    c3 = sqrt(1-(s3*s3));
    q2 = atan2(-(380+(380*s3))*x24 + (380*c3*z24) , (380*c3*x24)+((380*s3)+380));
    q3 = atan2(s3,c3);
    
    %define part 3    
    s1 = sin(q1);
    c1 = cos(q1);
    s23 = sin(q2 + q3);
    c23 = cos(q2 + q3);
    s_yaw = sin(yaw);
    c_yaw = cos(yaw);
    
    r11 = s23*c_yaw*s_pitch + c23*s1*s_yaw + c23*c_pitch*c1*c_yaw;
    r21 = c23*c_yaw*s_pitch - s23*s1*s_yaw - s23*c_pitch*c1*c_yaw;
    r31 = c_pitch*c_yaw*s1 - c1*s_yaw;
    c5 = sqrt(r11^2+r21^2);
    q5 = atan2(r31,c5);
    q4 = atan2(r21,r11);
    q = [q1;q2;q3;q4;q5]
else
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = 0
    q5 = 0
end
%% %% %% for low LV ลดรูปพจน์ซ้ำเเละเเทนค่าเเล้ว (v2)
q1 = atan2(y,x);

%define part 1
s_pitch = sin(pitch);
c_pitch = cos(pitch);

x24 = (x - 235*s_pitch) - 20.01;
z24 = (z - 235*c_pitch) - 275.99;

%define part 1
x24_z24_pow2 = ((x24*x24)+(z24*z24));

if (sqrt(x24_z24_pow2)<= (760))
    s3 = (x24_z24_pow2-288800)/(288800);
    c3 = sqrt(1-(s3*s3));
    q2 = atan2(-(380+(380*s3))*x24 + (380*c3*z24) , (380*c3*x24)+((380*s3)+380));
    q3 = atan2(s3,c3);
    
    %define part 2    
    s1 = sin(q1);
    c1 = cos(q1);
    s23 = sin(q2 + q3);
    c23 = cos(q2 + q3);
    s_yaw = sin(yaw);
    c_yaw = cos(yaw);
    c_yaw_s_pitch = c_yaw*s_pitch;
    s1_s_yaw = s1*s_yaw;
    c_pitch_c1_c_yaw = c_pitch*c1*c_yaw;
    
    
    r11 = (s23*c_yaw_s_pitch) + (c23*s1_s_yaw) + (c23*c_pitch_c1_c_yaw);
    r21 = (c23*c_yaw_s_pitch) - (s23*s1_s_yaw) - (s23*c_pitch_c1_c_yaw);
    r31 = (c_pitch*c_yaw*s1) - (c1*s_yaw);
    c5 = sqrt(r11^2+r21^2);
    q5 = atan2(r31,c5);
    q4 = atan2(r21,r11);
    q = [q1;q2;q3;q4;q5]
else
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = 0
    q5 = 0
end
%% FK
x = cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4));
y = sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4));
z = h1 + l2*sin(q2 + q3) + h2*cos(q2) - l3*cos(q2 + q3 + q4);
