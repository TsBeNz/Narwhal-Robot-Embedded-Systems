function [Pne] = FPK(q,l3)
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
%offset
h1 = 295.89;
h2 =380.00;
l1 =20;
l2 = 380.00;


Pne =  [l2*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + l3*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + l1*cos(q1) - h2*cos(q1)*sin(q2);
l3*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - l2*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + l1*sin(q1) - h2*sin(q1)*sin(q2);
                                                                h1 + l2*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + h2*cos(q2) - l3*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))]
 
end

