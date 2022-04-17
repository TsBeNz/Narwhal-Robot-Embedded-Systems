syms h1 h2 l1 l2 l3 q1 q2 q3 q4 q5 real
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
Hne=[1 0 0 0 ;
     0 1 0 0; 
     0 0 1 l3;
     0 0 0 1];
rho =[1; 1; 1;1;1];
gammabar = [1 1 -1];
q = [q1;q2;q3;q4;q5];
H = forwardKinematics(q,rho,DH,Hne);
[J,Je] = manipulatorJacobian(q,rho,DH,Hne)