function [q4,q5] = IOK2(R3e,gamma)
%IOK Summary of this function goes here
    r = @(i,j) R3e(i,j);
    c5 = gamma(3)*sqrt(R3e(1,1)^2+R3e(2,1)^2);
    q5 = atan2(R3e(3,1),c5);
    q4 = atan2(R3e(2,1)*gamma(3),r(1,1)*gamma(3));
end

