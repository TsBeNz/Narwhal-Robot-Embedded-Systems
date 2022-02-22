% %% ทำการ genTraj ของแต่ละ config var. q
Tmax = 5
% q1f= 0
% q2f= -0.201
% q3f= 0
% q4f= 0
% q5f= 0.0786
% 0.0840  -0.3396   -0.6426  0.9822  0

[q1VSt,t1f] = GenTraj(0,0.0840,0,Tmax);
[q2VSt,t2f] = GenTraj(0,0.4,0,Tmax);
[q3VSt,t3f] = GenTraj(0,0.3,0,Tmax);
[q4VSt,t4f] = GenTraj(0,-0.7,0,Tmax);
[q5VSt,t5f] = GenTraj(0,0,0,Tmax);
% tall = [t1f t2f t3f t4f t5f];
% T = max(tall)
% %% convert qiVSt  to timeseries for import to simulink
q1sim = timeseries(q1VSt,linspace(0,Tmax,numel(q1VSt)));
q2sim = timeseries(q2VSt,linspace(0,Tmax,numel(q2VSt)));
q3sim = timeseries(q3VSt,linspace(0,Tmax,numel(q3VSt)));
q4sim = timeseries(q4VSt,linspace(0,Tmax,numel(q4VSt)));
q5sim = timeseries(q5VSt,linspace(0,Tmax,numel(q5VSt)));

%% TEST IVK
% syms q1 q2 q3 q4 q5 wx wy wz vx vy vz h1 h2 l1 l2 l3 real
% taskspace = [wy; wz; vx; vy; vz];
% q =[q1 ;q2; q3 ;q4; q5];
% % % define parameters
% c =@(x) cos(x);
% s =@(x) sin(x);
% % % link length Update
% % h1= 275.99; % lasted
% % h2= 380;
% % l1= 20.01;
% % l2= 380;
% % l3= 235;
% DH =[0   0   h1   0;
%     l1 pi/2  0  pi/2;
%     h2  0    0  -pi/2;
%     l2  0    0    0;
%     0  pi/2  0    0];
% Hne=[1 0 0 0 ;
%      0 1 0 0; 
%      0 0 1 l3;
%      0 0 0 1];
% rho = [1;1;1;1;1];
% % run function
% H = forwardKinematics(q,rho,DH,Hne);
% [J,Je] = manipulatorJacobian(q,rho,DH,Hne);
% % Jrpy = [0  -s(q(1)) c(q(1))*s(q(2)+q(3)+q(4));
% %         0   c(q(1)) s(q(1))*s(q(2)+q(3)+q(4));
% %         1        0     c(q(2)+q(3)+q(4))];
% % Jerpy = Jrpy \ Je(1:3,:);
% % newJe = [Jerpy ; Je(4:6,:)];
% % ReducedJe = newJe(2:end,:);
% %%
% ReducedJe = Je(2:6,:);
% ReducedJe2 = vpa(ReducedJe,4)
% %%
% ReducedJe2 = [                                                                                                                                                                                                                                                                                                                                                      0,                                                                                                                                                                                  -cos(q1),                                                                                                            -cos(q1),                                                    -cos(q1), 0.5*cos(q2 - q1 + q3 + q4) - 0.5*cos(q1 + q2 + q3 + q4);
%                                                                                                                                                                                                                                                                                                                                                     1.0,                                                                                                                                                                                     0,                                                                                                               0,                                                       0,                                               -cos(q2 + q3 + q4);
% 117.5*cos(q1 + q2 + q3 + q4) - 117.5*cos(q2 - 1.0*q1 + q3 + q4)  - 20.01*sin(q1)   + 380.0*sin(q1)*sin(q2) + 380.0*sin(q1)*sin(q2)*sin(q3)  - 380.0*cos(q2)*cos(q3)*sin(q1), 117.5*cos(q2 - 1.0*q1 + q3 + q4) - 190.0*sin(q1 + q2 + q3) + 117.5*cos(q1 + q2 + q3 + q4) - 190.0*cos(q1 - 1.0*q2 + 6.123e-17) - 190.0*sin(q2 - 1.0*q1 + q3) - 190.0*cos(q1 + q2), 117.5*cos(q2 - 1.0*q1 + q3 + q4) - 190.0*sin(q1 + q2 + q3) + 117.5*cos(q1 + q2 + q3 + q4) - 190.0*sin(q2 - 1.0*q1 + q3), 117.5*cos(q2 - 1.0*q1 + q3 + q4) + 117.5*cos(q1 + q2 + q3 + q4),                                                                               0;
% 117.5*sin(q2 - 1.0*q1 + q3 + q4) + 117.5*sin(q1 + q2 + q3 + q4) + 20.01*cos(q1)   - 380.0*cos(q1)*sin(q2) - 1.425e-30*sin(q1)*sin(q2) + 380.0*cos(q1)*cos(q2)*cos(q3) - 380.0*cos(q1)*sin(q2)*sin(q3)  ,         190.0*cos(q1 + q2 + 1.571) + 190.0*cos(q1 + q2 + q3) - 117.5*sin(q2 - 1.0*q1 + q3 + q4) + 117.5*sin(q1 + q2 + q3 + q4) - 190.0*cos(q2 - 1.0*q1 + 1.571) - 190.0*cos(q2 - 1.0*q1 + q3), 190.0*cos(q1 + q2 + q3) - 117.5*sin(q2 - 1.0*q1 + q3 + q4) + 117.5*sin(q1 + q2 + q3 + q4) - 190.0*cos(q2 - 1.0*q1 + q3), 117.5*sin(q1 + q2 + q3 + q4) - 117.5*sin(q2 - 1.0*q1 + q3 + q4),                                                                               0;
%                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                          235.0*sin(q2 + q3 + q4) + 380.0*cos(q2 + 1.571) + 380.0*cos(q2 + q3),                                                                            235.0*sin(q2 + q3 + q4) + 380.0*cos(q2 + q3),                                         235.0*sin(q2 + q3 + q4),                                                                               0]
%  
% %%
% % inv_ReducedJe = simplify(inv(ReducedJe2))
% %%
% qv = simplify(ReducedJe2 \ taskspace)
% %%
% qv2 =simplify(vpa(qv,4))