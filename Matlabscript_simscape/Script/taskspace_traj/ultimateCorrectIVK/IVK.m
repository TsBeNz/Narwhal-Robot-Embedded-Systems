function qv = IVK(q,chi_dot) %q,qv5,chi_dot  % q,taskspace

h2 =380.00;
l1 =20;
l2 = 380.00;

Jv4 =[ -sin(q(1))*(l1 + l2*cos(q(2) + q(3)) - h2*sin(q(2))), -cos(q(1))*(l2*sin(q(2) + q(3)) + h2*cos(q(2))), -l2*sin(q(2) + q(3))*cos(q(1));
        cos(q(1))*(l1 + l2*cos(q(2) + q(3)) - h2*sin(q(2))), -sin(q(1))*(l2*sin(q(2) + q(3)) + h2*cos(q(2))), -l2*sin(q(2) + q(3))*sin(q(1));
                                                  0,            l2*cos(q(2) + q(3)) - h2*sin(q(2)),          l2*cos(q(2) + q(3))];

qvbar = Jv4 \ chi_dot;
qv = [qvbar(1); qvbar(2); qvbar(3); -qvbar(2)-qvbar(3)];

end


