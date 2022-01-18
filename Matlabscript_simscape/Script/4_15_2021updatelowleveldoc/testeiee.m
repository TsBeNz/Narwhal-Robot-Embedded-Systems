syms h1 h2 l1 l2 l3 x y z pitch yaw
%% Input
% link length Update
% h1= 275.99; % lasted
% h2= 380;
% l1= 20.01;
% l2= 380;
% l3= 235;
%Taskspace
% x =(l2+l1);
% y =0;
% z =(h1+h2-l3);
% pitch =-pi;
% yaw =pi;
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
%% 
R0e = [cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch);
                 sin(yaw),             cos(yaw),          0;
      cos(yaw)*sin(pitch), -sin(pitch)*sin(yaw), cos(pitch)];
pzz4 = [x;y;z]- (R0e * [0;0;l3])