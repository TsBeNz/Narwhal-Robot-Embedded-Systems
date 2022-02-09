function[qbar,shout] = IKnawhale2(P,rbar,gammabar,h1,h2,l1,l2,l3,DH)
    R0e = eul2rotm(rbar,'ZYZ');
    flag = 0;
    P004 = P - (R0e*[0;0;l3])
    [q1,q2,q3,flag] = IPK2(P,P004,gammabar,h1,h2,l1,l2,l3,flag); %P,P004,gammabar,h1,h2,l1,l2,l3,flag
    q03 = [q1 q2 q3];
    shout = flag;
    qbar = [q1;q2;q3];
    if flag == 0
        R03 = FOK(q03,DH);
        [q4,q5]= IOK2((R03.')*R0e,gammabar);
        qbar = [q1;q2;q3;q4;q5];
%         shout = 'Finished';
        shout = 0;
    else
%         shout = 'failed from h2 and l2 values';
        shout = 1;
        qbar = [0;0;0;0;0;0];
        msg = 'error link range.';
        error(msg)
    end
end

