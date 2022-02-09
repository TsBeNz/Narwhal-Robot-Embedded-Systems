syms q1 q2 q3 q4 q5 h1 h2 l1 l2 l3 real
qbar = [q1;q2;q3;q4;q5]
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
[H1,H2,H3,H4,H5,He] = FKnawhale(qbar,l3,DH)
fk_sym = simplify(He)
%%
h1= 275.99; % lasted
h2= 380;
l1= 20;
l2= 380;
l3= 268.23;
qbar = [0.5;0.2;0.8;0.1;0.3]
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
[H1,H2,H3,H4,H5,He] = FKnawhale(qbar,l3,DH)
% fk_sym = simplify(He)
%% solve eqs system
% syms x y z h1 h2 l1 l2 l3 q1 q2 q3 q4 q5 F K c1 s1 r31 r32 real
% eqns = [x == c1*(l1+l2*cos(q2+q3))-h2*sin(q2)+l3*sin(q2+q3+q4) ,y == s1*(l1+l2*cos(q2+q3))-h2*sin(q2)+l3*sin(q2+q3+q4) ...
%          ,z == h1+l2*sin(q2+q3)+h2*cos(q2)-l3*cos(q2+q3+q4), sin(q2+q3+q4)*cos(q5) == r31, -sin(q2+q3+q4)*sin(q5) == r32]
% vars =[q1 q2 q3 q4 q5]
% sol = solve(eqns,vars)
%%
k = cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)), l2*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + l3*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + l1*cos(q1) - h2*cos(q1)*sin(q2)
m =simplify(k)