function [qbar] = IKnarwhale4(P,rbar,gammabar,h1,h2,l1,l2,l3)
    R0e = eul2rotm(rbar(1:3),'ZYZ');
    r = gammabar(1)*sqrt(P(1)^2+P(2)^2);
    q1 = atan2(P(2)/gammabar(1), P(1)/gammabar(1));
    x4 = (r-l1)-l3*cos(rbar(2));
    z4 = (P(3)-h1)-l3*sin(rbar(2));
    if (sqrt(x4^2+24^2)<= (l2+h2)) && (sqrt(x4^2+z4^2)>=(l2-h2))
        c3 = (x4^2+z4^2-h2^2-l2^2)/ (2*h2*l2);
        s3 = gammabar(2)*sqrt(1-c3^2);
        q2 = atan2(z4,x4)- atan2(l2*s3, h2+l2*c3);

        q3 = atan2(s3,c3);
        q4 = rbar(2)-q2-q3;
        
        s5 = -R0e(3,2)/sin(rbar(2));
        c5 = R0e(3,1)/sin(rbar(2));
        q5 = atan2(s5,c5);
        qbar = [q1;q2;q3;q4;q5];
    else
       msg = 'Error occurred.';
       error(msg)
    end
end

