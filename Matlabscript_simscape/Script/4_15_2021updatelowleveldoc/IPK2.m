function [q1,q2,q3,flag] = IPK2(P,P004,gammabar,h1,h2,l1,l2,l3,flag)
    q1 = atan2(gammabar(1)*P(2),gammabar(1)*P(1));
    [q2,q3,flag]= RRIK2(P004,gammabar,h1,h2,l1,l2,flag);
end

