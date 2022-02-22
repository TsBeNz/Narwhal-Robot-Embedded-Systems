% q0 = 3
% qf = 5
% v0 = 0
% tf = 5 
% [qd,qv,qa,ti] = GenTraj(q0,qf,v0,tf)
% plot(qd,ti)
% plot(qd,ti)
% plot(qd,ti)
syms pi pf vi T vmax tau real
% A = [3*T -T^2;
%      -2    T]
% B = [pf-pi-(vi*T); 
%           -vi]
% K = A*B
% c2 = (vi*T - 3*(pi-pf+(vi*T)))/ T^2
% c3 = (2*pi - 2*pf+ (vi*T))/ T^3
c2 = -(3*pi - 3*pf + 2*T*vi)/T^2;
c3 = (2*pi - 2*pf + T*vi)/T^3;
% tau = (vi*T - 3*(pi-pf+(vi*T))*T) / (3*2*pi - 2*pf+ (vi*T))
% % vmax = vi + 2*c2*tau +3*c3*tau^2
% eq = vi + 2*c2*tau +3*c3*tau^2 -vmax == 0
% t = solve(eq,T)
eq = (2*c2)+(6*c3)*tau ==0
t_vmax = solve(eq,tau)
eq2 = vmax-(vi+2*c2*t_vmax+3*c3*t_vmax) == 0
vel = solve(eq2,vmax)

