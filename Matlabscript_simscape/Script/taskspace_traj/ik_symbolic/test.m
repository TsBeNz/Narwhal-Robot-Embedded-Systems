syms q1 q2 q3 q4 q5 h1 h2 l1 l2 l3 real
qbar = [q1;q2;q3;q4;q5]
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
[H1,H2,H3,H4,H5,He] = FKnawhale(qbar,l3,DH)
fk_sym = simplify(He)

%% solve eqs system
% syms x y z h1 h2 l1 l2 l3 q1 q2 q3 q4 q5 F K c1 s1 real
% eqns = [x == c1*(l1+l2*cos(q2+q3))-h2*sin(q2)+l3*sin(q2+q3+q4) ,y == s1*(l1+l2*cos(q2+q3))-h2*sin(q2)+l3*sin(q2+q3+q4) ...
%          ,z == h1+l2*sin(q2+q3)+h2*cos(q2)-l3*cos(q2+q3+q4)];
% vars =[q2 q3 q4]
% sol = solve(eqns,vars)