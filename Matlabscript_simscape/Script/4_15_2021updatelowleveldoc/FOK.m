function [R03] = FOK(q03,DH)% angles that rotate by each joint 
%FOK สำหรับหา FK ของ Orientation 3 dof ของarticulated robot เพื่อหาIK
%ที่เหลือต่อไป
    n = 3;
    c =@(x) cos(x);
    s =@(x) sin(x);
    p = [1;1;1]; % type of joint 
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
            Hj = Rotz(q03(i)); % Tranform by joint parameter
        else
            Hj = Transz(q03(i));
        end

        H= H*Tx*Rx*Tz*Rz*Hj; % Transform a frame respect to base frame
    end
        H03 = H;
        R03 = H03(1:3,1:3); % extract Rotm from H
end



