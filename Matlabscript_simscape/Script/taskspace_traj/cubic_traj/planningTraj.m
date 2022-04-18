function [q,v] = planningTraj(q0,q1,v0,v1,tf)
    t = linspace(0,tf,1000*(tf - 0));
    q = (2*q0*t.^3 - 2*q1*t.^3 + q0*tf.^3 - 3*q0*t.^2*tf + 3*q1*t.^2*tf + t*tf.^3*v0 + t.^3*tf*v0 + t.^3*tf*v1 - 2*t.^2*tf.^2*v0 - t.^2*tf.^2*v1)/tf^3;
    v = (6*q0*t.^2 - 6*q1*t.^2 + tf.^3*v0 - 4*t*tf^2*v0 + 3*t.^2*tf*v0 - 2*t*tf.^2*v1 + 3*t.^2*tf*v1 - 6*q0*t*tf + 6*q1*t*tf)/tf.^3;
end

