function [qbar,tod,tod2] = IKnarwhale4(P,rbar,gammabar,h1,h2,l1,l2,l3,DH)
    R0e = eul2rotm(rbar,'ZYZ')
    P4 = P - (R0e*[0;0;l3]);
%     x4 =P4(1)- l1
%     z4 =P4(3)- h1
    x4 = P4(1)-l1
    z4 = P4(3)-h1
%     r = gammabar(1)*sqrt(P(1)^2+P(2)^2);
    q1 = atan2(P(2)/gammabar(1), P(1)/gammabar(1));
    q5 = atan2(-R0e(3,2)/gammabar(3),R0e(3,1)/gammabar(3))
%     x4 = (r-l1)+l3*cos(rbar(2));
%     z4 = (P(3)-h1)+l3*sin(rbar(2));
%     if (sqrt(x4^2+z4^2)<= (l2+h2)) && (sqrt(x4^2+z4^2)>=(l2-h2))
        c3 = (x4^2+z4^2-h2^2-l2^2)/ (2*h2*l2)
        s3 = gammabar(2)*sqrt(1-c3^2)
        q2 = atan2(z4,x4)- atan2(l2*s3, h2+l2*c3)

        q3 = atan2(c3,s3)
%         flag = 0
%         P004 = [x4;0;z4]
%         [q2,q3,flag]=RRIK2(P004,gammabar,h1,h2,l1,l2,flag)
%         q4 = rbar(2)-q2-q3;
        tod = atan2(-R0e(3,2)/sin(q5),-R0e(3,3))
        tod2 = rbar(2)
        q4 = atan2(-R0e(3,2)/sin(q5),-R0e(3,3)) -q2-q3
%         R03 = FOK([q1;q2;q3],DH);
%         s5 = -R0e(3,2)/sin(rbar(2));
%         c5 = R0e(3,1)/sin(rbar(2));
%         q5 = atan2(s5,c5);
%         q5 = atan2(-R0e(3,2)/gammabar(3),R0e(3,1)/gammabar(3))
        qbar = [q1;q2;q3;q4;q5];
%     else
%        msg = 'Error occurred.';
%        error(msg)
%     end
end

