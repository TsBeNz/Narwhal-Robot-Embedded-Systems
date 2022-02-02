function [qbar] = IKnarwhale3(P,rbar,gammabar,h1,h2,l1,l2,l3)
    R0e = eul2rotm(rbar(1:3),'ZYZ')

    q1 = atan2(R0e(2,3)/gammabar(1) , R0e(1,3)/gammabar(1));
    q5 = atan2(-R0e(3,2)/gammabar(2) , R0e(3,1)/gammabar(2));

    c1 = cos(q1);
    s1 = sin(q1);
    c23 = ((P(1)-P(2) / c1-s1) - l1)/ l2;
    s23 = gammabar(3)*sqrt(1-c23^2);
    c234 = -R0e(3,3);
    c2 = (P(3)-h1 + (l3*c234)-(l2*s23))/h2
    s2 = gammabar(4)*sqrt(1-c2^2)
    q2 = atan2(s2,c2);

    q3 =atan2(s23,c23)-q2;

    c5 =cos(q5);
    s5 =sin(q5);

    q4 =atan2(-R0e(3,2)/s5, -R0e(3,3))-q2-q3;
    qbar = [q1;q2;q3;q4;q5]
end

