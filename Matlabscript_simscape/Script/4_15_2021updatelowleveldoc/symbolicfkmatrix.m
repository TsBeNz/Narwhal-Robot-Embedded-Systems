syms h1 h2 l1 l2 l3 q1 q2 q3 q4 q5 real
% define parameters
n = 5;
c =@(x) cos(x);
s =@(x) sin(x);
Hne=[1 0 0 0 ;
     0 1 0 0; 
     0 0 1 l3;
     0 0 0 1];
DH =[0   0   h1   0;
    l1 pi/2   0  pi/2;
     h2  0    0  -pi/2;
     l2  0    0   0;
     0  pi/2  0  0];
p = [1;1;1;1;1]; % type of joint
q = [q1;q2;q3;q4;q5]; % angles that rotate by each joint 
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
        H1 = floor(simplify(H))
    elseif i==2
        H2 = floor(simplify(H))
    elseif i==3
        H3 = floor(simplify(H))  
    elseif i==4
        H4 = floor(simplify(H))
    elseif i==5
        H5 = floor(simplify(H))
    end
end
    Hoe = H*Hne; % pose 0 -> e
    Hoe = simplify(Hoe)
%     Rotoe = simplify(Hoe(1:3,1:3)) % extract Rotm from H
    Rotoe = simplify(Hoe(1:3,1:3))
    Poe = simplify(Hoe(1:3,4))     % extract Position vector from H