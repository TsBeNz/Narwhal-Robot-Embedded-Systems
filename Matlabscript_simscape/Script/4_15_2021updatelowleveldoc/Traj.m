%จะลองทำ Traj. ใน joint space 
%1. กำหนด taskspace จุดเริ่มต้นและจุดปลายที่จะเคลื่อนที่ ไป 
%2. ทำการหา Inverse Kinematics ของจุดเริ่มต้นและจุดปลายจะได้ vector q
%   เริ่มต้นและจุดสุดท้าย 
%3. นำมาหา Traj. จากฟังก์ชันที่เบนซ์ให้มา แล้วทำ Timeseries เอาเข้า
%   simscape
% link length Update
h1= 275.99; % lasted
h2= 380;
l1= 20.01;
l2= 380;
l3= 235;
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
gammabar = [1 1 1];
%% กำหนด Taskspace เริ่มและปลาย 
taskI = [(l2+l1) 0 (h1+h2-l3) -pi pi];
taskF = [760 0   100  -pi pi];
%% ทำ IK ของจุดเริ่มต้นและ จุดสุดท้ายเพื่อให้ได้ qi กับ qf ของแต่ละ joint
[qbarI,shout] = IKnawhale2(taskI(1:3)',[0 taskI(4:5)],gammabar,h1,h2,l1,l2,l3,DH)
[qbarF,shout] = IKnawhale2(taskF(1:3)',[0 taskF(4:5)],gammabar,h1,h2,l1,l2,l3,DH)
%% คำนวณหา joint ที่ส่งผลให้ใช้เวลามากที่สุด
Tmax = 0
vmax = pi/2;
v0=0;
for i=1:5
    dq = qbarF(i) - qbarI(i);
    tf = abs(dq/((vmax-v0)/1.9));
    if tf > Tmax
        Tmax = tf;
    end
end
%%  ตัวอย่างการทำ Traj. quintic
q = [1 2 3 4];
qd_all=[];
qv_all=[];
qa_all=[];
itime = [2 2 3];
for  i = 1:numel(q)-1
    q0 = q(i);
    qf = q(i+1);
    v0 = 0;
    t0 = 0;
    tf = itime(i);
%    vmax = pi/2;
%     
%     dq = qf - q0;
%     tf = abs(dq/((vmax-v0)/1.9))
    
    t = linspace(t0,tf,1000*(tf - t0)); % Hz : 1000
    T = tf;
    
    ds = q0 - qf;
    tfv0 = tf*v0;
    
    co0 = q0
    co1 = v0
    co2 = 0
    co3 =  - (2*(5*ds + 3*tfv0))/tf^3
    co4 = (15*ds + 8*tfv0)/tf^4
    co5 =  - (3*(2*ds + tfv0))/tf^5
    
    % qd = co0 + (co1.*t) + (co2.*t.^2) + (co3.*t.^3) + (co4.*t.^4) + (co5.*t.^5)
    % qv = co1 + (2*co2.*t)+ (3*co3.*t.^2) + (4*co4.*t.^3) + (5*co5.*t.^4)
    qd = co0 + (co1.*t) + (co3.*t.^3) + (co4.*t.^4) + (co5.*t.^5)
    qv = co1 + (3*co3.*t.^2) + (4*co4.*t.^3) + (5*co5.*t.^4);
    qa = (6*co3.*t) + (12*co4.*t.^2) + (20*co5.*t.^3);
    qd_all{i} = qd
    qv_all{i} = qv
    qa_all{i} = qa
    
    
end
% hold on
for j = 1:numel(q)-1
    sumt =0
    if j == 1
     t_all = linspace(t(j), t(j+1), numel(qd_all{j}));
     hold on
     subplot(3,1,1)
     plot(t_all,qd_all{j})
     hold on
     subplot(3,1,2)
     plot(t_all,qv_all{j})
     hold on
     subplot(3,1,3)
     plot(t_all,qa_all{j})
     sumt = sumt + t(j)
    else
     t_all = linspace(t(j)+sumt, t(j+1), numel(qd_all{j}));
     hold on
     subplot(3,1,1)
     plot(t_all,qd_all{j})
     hold on
     subplot(3,1,2)
     plot(t_all,qv_all{j})
      hold on
     subplot(3,1,3)
     plot(t_all,qa_all{j})
     sumt = sumt + t(j)
    end
end
%     subplot(3,1,1)
%     plot(t_all,qd);
%     subplot(3,1,2)
%     plot(t,qv);
%     subplot(3,1,3)
%     plot(t,qa);
% %% ทำการ genTraj ของแต่ละ config var. q
% [q1VSt,t1f] = GenTraj(qbarI(1),qbarF(1),0,Tmax);
% [q2VSt,t2f] = GenTraj(qbarI(2),qbarF(2),0,Tmax);
% [q3VSt,t3f] = GenTraj(qbarI(3),qbarF(3),0,Tmax);
% [q4VSt,t4f] = GenTraj(qbarI(4),qbarF(4),0,Tmax);
% [q5VSt,t5f] = GenTraj(qbarI(5),qbarF(5),0,Tmax);
% % tall = [t1f t2f t3f t4f t5f];
% % T = max(tall)
% %% convert qiVSt  to timeseries for import to simulink
% q1sim = timeseries(q1VSt,linspace(0,Tmax,numel(q1VSt)));
% q2sim = timeseries(q2VSt,linspace(0,Tmax,numel(q2VSt)));
% q3sim = timeseries(q3VSt,linspace(0,Tmax,numel(q3VSt)));
% q4sim = timeseries(q4VSt,linspace(0,Tmax,numel(q4VSt)));
% q5sim = timeseries(q5VSt,linspace(0,Tmax,numel(q5VSt)));
