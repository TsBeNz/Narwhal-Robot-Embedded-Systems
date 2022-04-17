% h1 = 295.89;
% h2 =380.00;
% l1 =20;
% l2 = 380.00;
% l3= 269;
syms q1 q2 q3 q4 q5 h1 h2 l1 l2 l3 qd1 qd2 qd3 qd4 qd5 real
q = [q1;q2;q3;q4;q5]
qd = [qd1;qd2;qd3;qd4;qd5]
rho = [1,1,1,1,1]
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
Hne=[1 0 0 0 ;
     0 1 0 0; 
     0 0 1 l3;
     0 0 0 1];
[J,Je] = manipulatorJacobian(q,rho,DH,Hne)
twist = simplify(Je * qd)