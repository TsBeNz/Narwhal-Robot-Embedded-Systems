syms q1 q2 q3 q4 q5 h1 h2 l1 l2 l3 real
% define parameters
c =@(x) cos(x);
s =@(x) sin(x);
% h1= 251;s
% h2= 380;
% l1= 20;
% l2= 380;
% l3= 235;
% PI =sym(pi);
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
Hne=[1 0 0 0 ;
     0 1 0 0; 
     0 0 1 l3;
     0 0 0 1];
q=[q1;q2;q3;q4;q5];
rho = [1;1;1;1;1];
% run function
H = forwardKinematics(q,rho,DH,Hne);
[J,Je] = manipulatorJacobian(q,rho,DH,Hne)
Jrpy = [0  -s(q1) c(q1)*s(q2+q3+q4);
        0   c(q1) s(q1)*s(q2+q3+q4);
        1        0     c(q2+q3+q4)];
Jerpy = Jrpy \ Je(1:3,:);
% Jerpy = inv(Jrpy) * Je(1:3,:);
newJe = [Jerpy ; Je(4:6,:)];
ReducedJe = simplify(newJe(2:end,:));
DetRJe = simplify(det(ReducedJe))
% sad = solve(DetRJe,[q2,q3,q4],'ReturnConditions',true,'Real',true)