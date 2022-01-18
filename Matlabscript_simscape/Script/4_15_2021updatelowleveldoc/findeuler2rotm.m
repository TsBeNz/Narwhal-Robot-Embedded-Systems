syms roll pitch yaw z1 y z2 real
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
Rotmrpy = Rotz(roll)*Roty(pitch)*Rotz(yaw)
Rotmzyz= Rotz(z1)*Roty(y)*Rotz(z2)
matfunc = eul2rotm([0 -pi pi],'XYZ')

%% 
roll = 0;
pitch = -pi;
yaw = pi ;
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
%note that roll,pitch,yaw is fixed frame variable
Rotmzyz= Rotz(roll)*Roty(pitch)*Rotz(yaw) % เรียง [Z1 Y Z2]
ex = [ 0.0000   -0.7071   -0.7071;
       1.0000    0.0000         0;
      -0.0000    0.7071    0.7071]
r = atan2(ex(2,3),ex(1,3)) % Z2
p = acos(ex(3,3)) %Y
ya = -atan2(ex(2,1),ex(2,2)) %Z1
rpy = rotm2eul(Rotmzyz,'XYZ') 
%% แปลง Euler to rotm กรณีเป็น zyz  ก็คือ Rz(roll)*Ry(pitch)*Rz(yaw)
%% แปลง rotm to Euler กรณีเป็น zyz  ก็คือ ดุจัดเลยยย มี2เคสที่ไม่ unique solution
r11 = 0
r12 = -0.7071
r13 = -0.7071
r21 = 1
r22 = 0
r23 = 0
r31 = 0
r32 = 0.7071
r33 = 0.7071
rotmEx =[r11 r12 r13
         r21 r22 r23
         r31 r32 r33];
if (r33 > 1)
    if (r33 > -1)
        Y = acos(r33)
        Z0 = atan2(r23,r13)
        z1 = atan2(r32,-r31)
    else
        Y = pi
        Z0 = -atan2(r21,r22)
        z1 = 0
    end
else
     Y = 0
     Z0 = atan2(r21,r22)
     z1 = 0
end
