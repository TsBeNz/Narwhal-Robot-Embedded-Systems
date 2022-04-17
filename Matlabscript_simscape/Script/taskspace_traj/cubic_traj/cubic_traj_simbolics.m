clear
syms q0 q1 v0 v1 tf t real
% d = input(\ initial data = [q0,v0,q1,v1,t0,tf] = )
% q0 = 300; v0 = 20; q1 = 550; v1 = 0;
t0 = 0; 
% tf = 4.4305;
% t = linspace(t0,tf,100*(tf-t0));
% c = ones(size(t));
M = [   1 t0 t0^2 t0^3   ;
        0 1  2*t0 3*t0^2 ;
        1 tf tf^2 tf^3   ;
        0 1  2*tf 3*tf^2 ];
%%
b = [q0; v0; q1; v1];
a = inv(M)*b
%%
% qd = reference position trajectory
% vd = reference velocity trajectory
% ad = reference acceleration trajectory
%
qd = simplify(a(1) + a(2)*t +a(3)*t^2 + a(4)*t^3)
vd = simplify(a(2) +2*a(3)*t +3*a(4)*t^2)
ad = simplify(2*a(3) + 6*a(4)*t)
%%
t = (-2*a(3))/(6*a(4))
RHS = simplify(a(2) +2*a(3)*t +3*a(4)*t^2)
%%
syms vmax real
eq = RHS- vmax == 0
Tforvmax = simplify(solve(eq,tf))