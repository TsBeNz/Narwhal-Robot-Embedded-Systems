function [qd] = IVK(q,taskspace)
% define parameters
c =@(x) cos(x);
s =@(x) sin(x);
% link length Update
h1= 275.99; % lasted
h2= 380;
l1= 20.01;
l2= 380;
l3= 235;
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
Hne=[1 0 0 0 ;
     0 1 0 0; 
     0 0 1 l3;
     0 0 0 1];
rho = [1;1;1;1;1];
% run function
H = forwardKinematics(q,rho,DH,Hne);
[J,Je] = manipulatorJacobian(q,rho,DH,Hne);
Jrpy = [0  -s(q(1)) c(q(1))*s(q(2)+q(3)+q(4));
        0   c(q(1)) s(q(1))*s(q(2)+q(3)+q(4));
        1        0     c(q(2)+q(3)+q(4))];
Jerpy = Jrpy \ Je(1:3,:);
newJe = [Jerpy ; Je(4:6,:)];
ReducedJe = newJe(2:end,:);
qd = ReducedJe*taskspace;
end

