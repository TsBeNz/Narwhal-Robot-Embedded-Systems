%ทำ Trajectory ใน Taskspace
%1.กำหนด goal ใน Taskspace และ ตำแหน่งป้อนกลับใน Taskspace เป็น input
%ของการทดสอบนี้
%2.จากนั้น Traj จะ output ความเร็วและตำแหน่งใน taskspace ในแต่ละเวลา
%3.จากนั้น นำ ตำแหน่ง Taskspace ไปผ่านฟังก์ชัน IPK จะได้ตำแหน่งใน
%configuration space
%4.จากนั้น นำ ความเร็ว Taskspace ไปผ่านฟังก์ชัน IVK จะได้ความเร็วแต่ละjointใน
%configuration space
% link length Update
h1= 275.99/1000; % lasted
h2= 380/1000;
l1= 20.01/1000;
l2= 380/1000;
l3= 235/1000;
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
gammabar = [1 1 1];
%%  กำหนด Input
Xviapoint = [(l2+l1)         0      (h1+h2-l3)   -pi    pi;
              300/1000  100/1000     350/1000    -pi/2  pi;
              200/1000  200/1000     300/1000     0     pi/2 ]; %3 via points
viapoint [100]
% Xviapoint = Xviapoint/10.0;
% Xgoal = [500 0   260  -pi pi]
itime = [10 12]
%% นำเข้าไปคำนวณ Traj.
% Tmax = 5
n = size(Xviapoint)
for i = 1: n(1)-1
    [Xd1{i},Xv1{i}] = GenTraj(Xviapoint(i,1),Xviapoint(i+1,1),0,itime(i));
    [Xd2{i},Xv2{i}] = GenTraj(Xviapoint(i,2),Xviapoint(i+1,2),0,itime(i));
    [Xd3{i},Xv3{i}] = GenTraj(Xviapoint(i,3),Xviapoint(i+1,3),0,itime(i));
    [Xd4{i},Xv4{i}] = GenTraj(Xviapoint(i,4),Xviapoint(i+1,4),0,itime(i));
    [Xd5{i},Xv5{i}] = GenTraj(Xviapoint(i,5),Xviapoint(i+1,5),0,itime(i));
end

% plot taskspace traj.
% sumt =0
% for j = 1:numel(itime)
%     if j == 1
%      t_all = linspace(0, itime(j), numel(Xd1{j}));
%     hold on
%      subplot(3,1,1)
%      plot(t_all,Xd1{j},'r')
%      hold on
%      plot(t_all,Xd2{j},'g')
%      hold on
%      plot(t_all,Xd3{j},'b')
%      hold on
%      plot(t_all,Xd4{j},'c')
%      hold on
%      plot(t_all,Xv5{j},'m')

%      hold on
%      subplot(3,1,2)
%      hold on
%      plot(t_all,Xv1{j},'r')
%      hold on
%      plot(t_all,Xv2{j},'g')
%      hold on
%      plot(t_all,Xv3{j},'b')
%      hold on
%      plot(t_all,Xv4{j},'c')
%      hold on
%      plot(t_all,Xv5{j},'m')
%      hold on
%      subplot(3,1,3)
%      plot(t_all,X1{j})
%      sumt = sumt + itime(j)
%     else
%      t_all = linspace(sumt, itime(j)+sumt, numel(Xd1{j}));
%      hold on
%      subplot(3,1,1)
%      plot(t_all,Xd1{j},'r')
%      hold on
%      plot(t_all,Xd2{j},'g')
%      hold on
%      plot(t_all,Xd3{j},'b')
%      hold on
%      plot(t_all,Xd4{j},'c')
%      hold on
%      plot(t_all,Xv5{j},'m')
% 
%      hold on
%      subplot(3,1,2)
%      hold on
%      plot(t_all,Xv1{j},'r')
%      hold on
%      plot(t_all,Xv2{j},'g')
%      hold on
%      plot(t_all,Xv3{j},'b')
%      hold on
%      plot(t_all,Xv4{j},'c')
%      hold on
%      plot(t_all,Xv5{j},'m')
%       hold on
%      subplot(3,1,3)
%      plot(t_all,Xa_all{j})
%      sumt = sumt + itime(j)
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% end plot taskspace traj
% tall = [t1f t2f t3f t4f t5f]
% Xd = [Xd1; Xd2; Xd3; Xd4; Xd5 ]
% Xv = [Xv1; Xv2; Xv3; Xv4; Xv5 ]
% l = size(Xd1)
% time = linspace(0,Tmax,l(2))
% plot(time,Xv)
%% ได้ Xti และ Xdti แต่ละเวลาออกมาจากนั้นแยกเข้าคำนวณ IVK และ IPK
%IPK ถ้าviapoints ห่างเกินอาจะทำให้เกิดการ singularity / แก้ inverse ไม่ได้
%แก้ได้โดยการทำให้ viapoint ถี่ขึ้น
n= size(Xd1);
for i =1:n(2)
    for j = 1:numel(Xd1{i})
        position = [Xd1{i}(j) Xd2{i}(j) Xd3{i}(j)];
        orientation = [0 [Xd4{i}(j) Xd5{i}(j)]];
        [qbar(j,:),shout] = IKnawhale2(position',orientation,gammabar,h1,h2,l1,l2,l3,DH);
%         qbar(:,i)=qbari';
    end
        qbar_viapoint{i}=qbar;
end


%% IVK
qvbar=[]
for i =1:n(2)
    for j = 1:numel(Xv1{i})
        Xv = [Xv1{i}(j); Xv2{i}(j); Xv3{i}(j); Xv4{i}(j); Xv5{i}(j) ]
        [qvbar(j,:)] = IVK(qbar_viapoint{i}(i,:)',Xv);
%     qvbar(:,i)=qvbari';
    end
       qvbar_viapoint{i}=qvbar
end
% plot joint value
sumt =0
for j = 1:numel(itime)
    if j == 1
     t_all = linspace(0, itime(j), numel(Xd1{j}));
%      karn = size(qbar_viapoint{j}(:,1))
     hold on
     subplot(3,1,1)
     plot(t_all,qbar_viapoint{j}(:,1),'r')
     title('Joint Position')
     hold on
     plot(t_all,qbar_viapoint{j}(:,2),'g')
     hold on
     plot(t_all,qbar_viapoint{j}(:,3),'b')
     hold on
     plot(t_all,qbar_viapoint{j}(:,4),'c')
     hold on
     plot(t_all,qbar_viapoint{j}(:,5),'m')
     hold on

     subplot(3,1,2)
     plot(t_all,qvbar_viapoint{j}(:,1),'r')
     title('Joint Velocity')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,2),'g')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,3),'b')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,4),'c')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,5),'m')

     sumt = sumt + itime(j)
    else
     t_all = linspace(sumt, itime(j)+sumt, numel(Xd1{j}));
%      karn = size(qbar_viapoint{j}(1,:)hold on
     subplot(3,1,1)
     plot(t_all,qbar_viapoint{j}(:,1),'r')
     hold on
     plot(t_all,qbar_viapoint{j}(:,2),'g')
     hold on
     plot(t_all,qbar_viapoint{j}(:,3),'b')
     hold on
     plot(t_all,qbar_viapoint{j}(:,4),'c')
     hold on
     plot(t_all,qbar_viapoint{j}(:,5),'m')

     hold on
     subplot(3,1,2)
     plot(t_all,qvbar_viapoint{j}(:,1),'r')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,2),'g')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,3),'b')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,4),'c')
     hold on
     plot(t_all,qvbar_viapoint{j}(:,5),'m')
     hold on
     sumt = sumt + itime(j)
    end
end
% for i = 1:numel(itime)
%     for j = 1:5
%         ind = find(qvbar_viapoint{i}(:,j) == max(qvbar_viapoint{i}(:,j)))
%         hold on
%         plot(t_all(ind),qvbar_viapoint{i}(ind,j),'*k')
%     end
% end
%% ทดสอบนอกเรื่อง
% syms q1 q2 q3 q4 q5 wx wy wz vx vy vz h1 h2 l1 l2 l3 real
% c =@(x) cos(x);
% s =@(x) sin(x);
% % link length Update
% h1= 275.99; % lasted
% h2= 380;
% l1= 20.01;
% l2= 380;
% l3= 235;
% DH =[0   0   h1   0;
%     l1 pi/2  0  pi/2;
%     h2  0    0  -pi/2;
%     l2  0    0    0;
%     0  pi/2  0    0];
% q=[q1;q2;q3;q4;q5];
% rho = [1;1;1;1;1];
% Hne=[1 0 0 0 ;
%      0 1 0 0; 
%      0 0 1 l3;
%      0 0 0 1];
%  xi = [wx; wy; wz; vx; vy; vz];
%%  หา Jacobian ก่อน เป็น Jacobian ที่คูณ Jrpy แล้วเนื่องจากมี input เป็น rpy ด้วย
% H = forwardKinematics(q,rho,DH,Hne);
% [J,Je] = manipulatorJacobian(q,rho,DH,Hne)
% Jrpy = [0  -s(q1) c(q1)*s(q2+q3+q4);
%         0   c(q1) s(q1)*s(q2+q3+q4);
%         1        0     c(q2+q3+q4)];
% Jerpy = Jrpy \ Je(1:3,:);
% newJe = [Jerpy ; Je(4:6,:)]
% ReducedJe = simplify(newJe(2:end,:));
% Reducedxi = xi(2:end);
% DetRJe = simplify(det(ReducedJe));