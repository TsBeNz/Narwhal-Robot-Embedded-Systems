function [twist] = FVK(q,qd,l3)
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
% joint vel
qd1 = qd(1);
qd2 = qd(2);
qd3 = qd(3);
qd4 = qd(4);
%offset
h2 =380.00;
l1 =20;
l2 = 380.00;
% l3= 269;
twist =[ l3*qd4*cos(q2 + q3 + q4)*cos(q1) - qd3*cos(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)) - qd1*sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4)) - qd2*cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2) - l3*cos(q2 + q3 + q4));
qd1*cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4)) - qd3*sin(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)) - qd2*sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2) - l3*cos(q2 + q3 + q4)) + l3*qd4*cos(q2 + q3 + q4)*sin(q1);
                                                                                                 qd2*(l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4)) + qd3*(l2*cos(q2 + q3) + l3*sin(q2 + q3 + q4)) + l3*qd4*sin(q2 + q3 + q4)];
 
end

