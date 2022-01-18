function  [q2,q3,flag]= RRIK2(P004,gammabar,h1,h2,l1,l2,flag)
    P023 = P004-[l1 ;0 ;h1];
    x24 = P023(1);
    z24 = P023(3);
%     if (sqrt(x24^2+z24^2)<= (l2+h2)) && (sqrt(x24^2+z24^2)>=(l2-h2))
        s3 = (x24^2+z24^2-h2^2-l2^2)/(2*l2*h2);
        c3 = gammabar(2)*sqrt(1-s3^2);
        q2 = atan2(-(h2+l2*s3)*x24 + (l2*c3*z24) , (l2*c3*x24)+(l2*s3+h2));
        q3 = atan2(s3,c3);
%     else
%         flag = 1; % สั่งเกินระยะแขน เกิดท่าประหลาดตา
%         q2 =0;
%         q3=0;
%     end 
end

