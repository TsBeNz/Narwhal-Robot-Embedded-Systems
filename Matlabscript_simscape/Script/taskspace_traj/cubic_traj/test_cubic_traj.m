q0 = 0
qf = 100
v0 = 0
vmax =30
[qd,qv,tf] = cubic_traj(q0,qf,v0,vmax)
t = linspace(0,tf,1000*(tf - 0));
plot(t,qd)
plot()