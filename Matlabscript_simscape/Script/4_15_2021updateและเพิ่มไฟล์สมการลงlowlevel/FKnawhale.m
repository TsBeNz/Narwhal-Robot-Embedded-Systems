function [H1,H2,H3,H4,H5,He] = FKnawhale(qbar,l3,DH)
n = 5;
c =@(x) cos(x);
s =@(x) sin(x);
Hne=[1 0 0 0 ;
     0 1 0 0; 
     0 0 1 l3;
     0 0 0 1];
p = [1;1;1;1;1]; % type of joint
q = [qbar(1);qbar(2);qbar(3);qbar(4);qbar(5)]; % angles that rotate by each joint 
% define Translation & Rotation function 
Transx =@(a) [1 0 0 a;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];
Rotx =@(b) [1  0     0    0;
            0 c(b) -s(b)  0;
            0 s(b)  c(b)  0;
            0  0     0    1];
Transz =@(c) [1 0 0 0;
              0 1 0 0;
              0 0 1 c;
              0 0 0 1];
Rotz =@(d) [c(d) -s(d) 0 0;
            s(d)  c(d) 0 0;
            0      0   1 0;
            0      0   0 1];
H = eye(4); % start with 4*4 Identity matrix
for i = 1 : n
    % Transform by fixed parameters
    Tx = Transx(DH(i,1));  
    Rx = Rotx(DH(i,2));
    Tz = Transz(DH(i,3));
    Rz = Rotz(DH(i,4));
    if p(i)==1
        Hj = Rotz(q(i)); % Tranform by joint parameter
    else
        Hj = Transz(q(i));
    end
    
    H= H*Tx*Rx*Tz*Rz*Hj; % Transform a frame respect to base frame
    if i==1
        H1 = H;
    elseif i==2
        H2 = H;
    elseif i==3
        H3 = H;  
    elseif i==4
        H4 = H;
    elseif i==5
        H5 = H;
    end
    
end
    He = H*Hne; % pose 0 -> e
%     Rotoe = simplify(Hoe(1:3,1:3)) % extract Rotm from H
    Rotoe = He(1:3,1:3);
    Poe = He(1:3,4);     % extract Position vector from H
end

