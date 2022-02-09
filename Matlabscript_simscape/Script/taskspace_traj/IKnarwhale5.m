function [qbar] = IKnarwhale5(chi,gammabar,h1,h2,l1,l2,l3)
    q1 = atan2(gammabar(1)*chi(2),gammabar(1)*chi(1));
    % RRIK
    r = gammabar(2)*sqrt(chi(1)^2 + chi(2)^2)
    z24 = chi(3)+l3-h1
    x24 = r-l1
    if (sqrt(x24^2+z24^2)<= (l2+h2)) && (sqrt(x24^2+z24^2)>=(l2-h2))

%         [q2,q3] = RRIK(x24,z24,h2,l2,gammabar(3))
        c2 = (x24^2+z24^2-h2^2-l2^2)/(2*h2*l2)
        s2 = gammabar(3)*sqrt(1-c2^2)
        q2 = (atan2(z24,x24)-atan2(l2*s2,h2+l2*c2))-(pi/2) 
        q3 = atan2(s2,c2)+(pi/2)
    else
        % สั่งเกินระยะแขน เกิดท่าประหลาด
        msg = 'error link range.';
        error(msg)
    end 
    q4 = -q2-q3;
    q5 = chi(4);
    qbar = [q1;q2;q3;q4;q5];
end

