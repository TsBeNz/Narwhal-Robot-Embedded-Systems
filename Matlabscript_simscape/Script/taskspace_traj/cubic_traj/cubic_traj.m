function [qd,qv,tf] = cubic_traj(q0,qf,v0,vmax)
    tf = (1.5*(qf-qi))/ vmax
    c0 = q0;
    c1 = v0;
    c2 = (3*(qf-q0))/tf^2;
    c3 = (-2*(qf-q0))/tf^3;
    t = linspace(0,tf,1000*(tf - 0));
    qd = c0 + c1*t +c2*t^2 +c3*t^3;
    qv = c1 + 2*c2*t + 3*c3*t^2;
end

