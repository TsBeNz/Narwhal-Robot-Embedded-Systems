function [q1,q2] = RRIK(x,y,l1,l2,gamma)
    c2 = (x^2+y^2-l1^2-l2^2)/(2*l1*l2)
    s2 = gamma*sqrt(1-c2^2)
    q1 = (atan2(y,x)-atan2(l2*s2,l1+l2*c2))-(pi/2)
    q2 = atan2(s2,c2)+(pi/2)
end

