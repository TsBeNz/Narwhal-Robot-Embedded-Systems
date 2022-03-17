%ทำ Trajectory ใน Taskspace
%1.กำหนด goal ใน Taskspace และ ตำแหน่งป้อนกลับใน Taskspace เป็น input
%ของการทดสอบนี้
%2.จากนั้น Traj จะ output ความเร็วและตำแหน่งใน taskspace ในแต่ละเวลา
%3.จากนั้น นำ ตำแหน่ง Taskspace ไปผ่านฟังก์ชัน IPK จะได้ตำแหน่งใน
%configuration space
%4.จากนั้น นำ ความเร็ว Taskspace ไปผ่านฟังก์ชัน IVK จะได้ความเร็วแต่ละjointใน
%configuration space
% link length Update
% h1= 275.99; 
% h2= 380;
% l1= 20.01;
% l2= 380;
% l3= 268.23;
% lasted
h1 = 295.89;
h2 =380.00;
l1 =20;
l2 = 380.00;
l3= 269;
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
gammabar = [1 1 -1];
%%  กำหนด Input
Xviapoint = [(l2+l1)         0      (h1+h2-l3) 0;
              550           20         30      0;
              200            0         50      0 ]; %3 via points
% viapoint [100]
% Xviapoint = Xviapoint/10.0;
% Xgoal = [500 0   260  -pi pi]
v_max = 80 % mm/s 60+20
delta_vp = []
n_vp =size(Xviapoint)
for i=1:n_vp(1)-1
    for j=1:4
        delta_vp(i,j) = abs(Xviapoint(i+1,j)-Xviapoint(i,j))
    end
end
itime = [(2*max(delta_vp(1))/v_max)+2 (2*max(delta_vp(2))/v_max)+2]
%% นำเข้าไปคำนวณ Traj.
% Tmax = 5
n = size(Xviapoint)
for i = 1: n(1)-1
    [Xd1{i},Xv1{i}] = GenTraj(Xviapoint(i,1),Xviapoint(i+1,1),0,itime(i));
    [Xd2{i},Xv2{i}] = GenTraj(Xviapoint(i,2),Xviapoint(i+1,2),0,itime(i));
    [Xd3{i},Xv3{i}] = GenTraj(Xviapoint(i,3),Xviapoint(i+1,3),0,itime(i));
    [Xd4{i},Xv4{i}] = GenTraj(Xviapoint(i,4),Xviapoint(i+1,4),0,itime(i));
end

% %plot taskspace traj.
% sumt =0
% for j = 1:numel(itime)
%     if j == 1
%      t_all = linspace(0, itime(j), numel(Xd1{j}));
%     hold on
%      subplot(2,1,1)
%      title('Position Trajectory')
%      xlabel('time(sec)')
%      ylabel('position(mm)')
%      plot(t_all,Xd1{j},'r')
%      hold on
%      plot(t_all,Xd2{j},'g')
%      hold on
%      plot(t_all,Xd3{j},'b')
% %      hold on
% %      plot(t_all,Xd4{j},'c')
% %      hold on
% %      plot(t_all,Xv5{j},'m')
%      legend('x','y','z')
%      hold on
%      subplot(2,1,2)
%      title('Velocity Trajectory')
%      xlabel('time(sec)')
%      ylabel('velocity(mm/s)')
%      hold on
%      plot(t_all,Xv1{j},'r')
%      hold on
%      plot(t_all,Xv2{j},'g')
%      hold on
%      plot(t_all,Xv3{j},'b')
%      hold on
% %      plot(t_all,Xv4{j},'c')
% %      hold on
% %      plot(t_all,Xv5{j},'m')
% %      hold on
%      legend('x','y','z')
% %      subplot(3,1,3)
% %      plot(t_all,X1{j})
%      sumt = sumt + itime(j)
%     else
%      t_all = linspace(sumt, itime(j)+sumt, numel(Xd1{j}));
%      hold on
%      subplot(2,1,1)
%      title('Position Trajectory')
%      xlabel('time(sec)')
%      ylabel('position(mm)')
%      plot(t_all,Xd1{j},'r')
%      hold on
%      plot(t_all,Xd2{j},'g')
%      hold on
%      plot(t_all,Xd3{j},'b')
%      hold on
% %      plot(t_all,Xd4{j},'c')
% %      hold on
% %      plot(t_all,Xv5{j},'m')
% 
% %      hold on
%      legend('x','y','z')
%      subplot(2,1,2)
%      title('Velocity Trajectory')
%      xlabel('time(sec)')
%      ylabel('velocity(mm/s)')
%      hold on
%      plot(t_all,Xv1{j},'r')
%      hold on
%      plot(t_all,Xv2{j},'g')
%      hold on
%      plot(t_all,Xv3{j},'b')
%      hold on
% %      plot(t_all,Xv4{j},'c')
% %      hold on
% %      plot(t_all,Xv5{j},'m')
% %       hold on
% %      subplot(3,1,3)
% %      plot(t_all,Xa_all{j})
%      legend('x','y','z')
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
        position = [Xd1{i}(j) Xd2{i}(j) Xd3{i}(j) Xd4{i}(j)];
        [qbar(j,:)] = IKnarwhale5(position',gammabar,l3);
%         qbar(:,i)=qbari';
    end
        qbar_viapoint{i}=qbar;
end


%% IVK
qvbar=[]
for i =1:n(2)
    for j = 1:numel(Xv1{i})
        Xv = [Xv1{i}(j); Xv2{i}(j); Xv3{i}(j) ]
        [qvbar(j,:)] = IVK(qbar_viapoint{i}(i,:)',0,Xv);
%     qvbar(:,i)=qvbari';
    end
       qvbar_viapoint{i}=qvbar
end
%%
% plot joint value
sumt =0
for j = 1:numel(itime)
    if j == 1
     t_all = linspace(0, itime(j), numel(Xd1{j}));
     karn = size(qbar_viapoint{j}(:,1))
     hold on
     subplot(2,1,1)
     title('Position Trajectory')
     xlabel('time(sec)')
     ylabel('position(rad)')
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
     legend('j1','j2','j3','j4','j5')
     subplot(2,1,2)
     title('Velocity Trajectory')
     xlabel('time(sec)')
     ylabel('velocity(rad/s)')
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
     legend('j1','j2','j3','j4','j5')
     sumt = sumt + itime(j)
    else
     t_all = linspace(sumt, itime(j)+sumt, numel(Xd1{j}));
     %karn = size(qbar_viapoint{j}(1,:)hold on
     subplot(2,1,1)
     title('Position Trajectory')
     xlabel('time(sec)')
     ylabel('position(rad)')
     plot(t_all,qbar_viapoint{j}(:,1),'r')
     hold on
     plot(t_all,qbar_viapoint{j}(:,2),'g')
     hold on
     plot(t_all,qbar_viapoint{j}(:,3),'b')
     hold on
     plot(t_all,qbar_viapoint{j}(:,4),'c')
     hold on
     plot(t_all,qbar_viapoint{j}(:,5),'m')
     legend('j1','j2','j3','j4','j5')
     hold on
     subplot(2,1,2)
     title('Velocity Trajectory')
     xlabel('time(sec)')
     ylabel('velocity(rad/s)')
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
     legend('j1','j2','j3','j4','j5')
     sumt = sumt + itime(j)
    end
end
%%%%% end plot
% for i = 1:numel(itime)
%     for j = 1:5
%         ind = find(qvbar_viapoint{i}(:,j) == max(qvbar_viapoint{i}(:,j)))
%         hold on
%         plot(t_all(ind),qvbar_viapoint{i}(ind,j),'*k')
%     end
% end
%% input simscape
% q1sim = timeseries(qbar(:,1),linspace(0,10,numel(qbar(:,1))));
% q2sim = timeseries(qbar(:,2),linspace(0,10,numel(qbar(:,2))));
% q3sim = timeseries(qbar(:,3),linspace(0,10,numel(qbar(:,3))));
% q4sim = timeseries(qbar(:,4),linspace(0,10,numel(qbar(:,4))));
% q5sim = timeseries(qbar(:,5),linspace(0,10,numel(qbar(:,5))));