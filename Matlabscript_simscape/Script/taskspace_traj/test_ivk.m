syms q1 q2 l1 h2 real
J3 = [                         0,             sin(q1),  sin(q1), 0, 0;
                               0,            -cos(q1), -cos(q1), 0, 0;
                               1,                   0,        0, 0, 0;
      -sin(q1)*(l1 - h2*sin(q2)), -h2*cos(q1)*cos(q2),        0, 0, 0;
       cos(q1)*(l1 - h2*sin(q2)), -h2*cos(q2)*sin(q1),        0, 0, 0;
                               0,         -h2*sin(q2),        0, 0, 0]

 
Je =[                                                                           0,                                                               sin(q1),                                           sin(q1),                      sin(q1), sin(q2 + q3 + q4)*cos(q1);
                                                                           0,                                                              -cos(q1),                                          -cos(q1),                     -cos(q1), sin(q2 + q3 + q4)*sin(q1);
                                                                           1,                                                                     0,                                                 0,                            0,        -cos(q2 + q3 + q4);
- sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) - l3*sin(q2 + q3 + q4)*sin(q1), l3*cos(q2 + q3 + q4)*cos(q1) - cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -cos(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*cos(q1),                         0;
  cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) + l3*sin(q2 + q3 + q4)*cos(q1), l3*cos(q2 + q3 + q4)*sin(q1) - sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -sin(q1)*(l2*sin(q2 + q3) - l3*cos(q2 + q3 + q4)), l3*cos(q2 + q3 + q4)*sin(q1),                         0;
                                                                           0,                   l2*cos(q2 + q3) - h2*sin(q2) + l3*sin(q2 + q3 + q4),            l2*cos(q2 + q3) + l3*sin(q2 + q3 + q4),         l3*sin(q2 + q3 + q4),                         0]
Jev3 = [- sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) , l3*cos(q1) - cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -cos(q1)*(l2*sin(q2 + q3) - l3);
  cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2))        , l3*sin(q1) - sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -sin(q1)*(l2*sin(q2 + q3) - l3);
                                                  0  ,  l2*cos(q2 + q3) - h2*sin(q2)                      ,  l2*cos(q2 + q3) ]

Jv5 =[ -sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)), -cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -l2*sin(q2 + q3)*cos(q1);
        cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)), -sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -l2*sin(q2 + q3)*sin(q1);
                                                  0,            l2*cos(q2 + q3) - h2*sin(q2),          l2*cos(q2 + q3)]
% Jv3 =[-sin(q1)*(l1 - h2*sin(q2)), -h2*cos(q1)*cos(q2),        0, ;
%        cos(q1)*(l1 - h2*sin(q2)), -h2*cos(q2)*sin(q1),        0, ;
%                                0,         -h2*sin(q2),        0, ]  
Jv3 =[                         1,                   0,        0,  -1;
      -sin(q1)*(l1 - h2*sin(q2)), -h2*cos(q1)*cos(q2),        0,  0;
       cos(q1)*(l1 - h2*sin(q2)), -h2*cos(q2)*sin(q1),        0,  0;
                               0,         -h2*sin(q2),        0,  0]
Jve4=[                                             1,                                                   0,                               0,                 -1;
      - sin(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) , l3*cos(q1) - cos(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -cos(q1)*(l2*sin(q2 + q3) - l3),                  0;
        cos(q1)*(l1 + l2*cos(q2 + q3) - h2*sin(q2)) , l3*sin(q1) - sin(q1)*(l2*sin(q2 + q3) + h2*cos(q2)), -sin(q1)*(l2*sin(q2 + q3) - l3),                  0;
                                                   0,                       l2*cos(q2 + q3) - h2*sin(q2) ,                l2*cos(q2 + q3) ,                  0]
inv_Jv3 = simplify(inv(Jv3))
inv_Jv5 = simplify(inv(Jv5))
% inv_Jev3 = simplify(inv(Jev3))
%% IVK 3 dof แรก
syms vx vy vz q2d q3d q5d real
chi_dot=[vx;vy;vz]
qvbar = simplify(Jev3 \ chi_dot)  %qvbar = [q1d q2d q3d]
q4d = -q2d-q3d
q5d = q5d