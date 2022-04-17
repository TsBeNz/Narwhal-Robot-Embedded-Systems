q0 = 10
q1 = 50
tf = 5
v0 =0;
v1 =0;
[q,v,t] = planningTraj(q0,q1,v0,v1,tf)
subplot(2,1,1)
plot(t,q)
subplot(2,1,2)
plot(t,v)