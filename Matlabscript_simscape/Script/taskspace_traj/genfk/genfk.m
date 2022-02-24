% h1 = 295.89;
% h2 =380.00;
% l1 =20;
% l2 = 380.00;
% l3= 268.23;
syms q1 q2 q3 q4 q5 h1 h2 l1 l2 l3 real
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
q = [q1 q2 q3 q4 q5]
[H1,H2,H3,H4,H5,He] = FKnawhale(q,l3,DH)  