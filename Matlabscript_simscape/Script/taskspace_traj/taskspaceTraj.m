%ทำ Trajectory ใน Taskspace
%1.กำหนด goal ใน Taskspace และ ตำแหน่งป้อนกลับใน Taskspace เป็น input
%ของการทดสอบนี้
%2.จากนั้น Traj จะ output ความเร็วและตำแหน่งใน taskspace ในแต่ละเวลา
%3.จากนั้น นำ ตำแหน่ง Taskspace ไปผ่านฟังก์ชัน IPK จะได้ตำแหน่งใน
%configuration space
%4.จากนั้น นำ ความเร็ว Taskspace ไปผ่านฟังก์ชัน IVK จะได้ความเร็วแต่ละjointใน
%configuration space
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
%%  กำหนด Input
Xfeedback = [(l2+l1) 0 (h1+h2-l3) -pi pi]
Xgoal = [500 0   260  -pi pi]
%% นำเข้าไปคำนวณ Traj.
Tmax = 2
[Xd1,Xv1,t1f] = GenTraj(Xfeedback(1),Xgoal(1),0,Tmax);
[Xd2,Xv2,t2f] = GenTraj(Xfeedback(2),Xgoal(2),0,Tmax);
[Xd3,Xv3,t3f] = GenTraj(Xfeedback(3),Xgoal(3),0,Tmax);
[Xd4,Xv4,t4f] = GenTraj(Xfeedback(4),Xgoal(4),0,Tmax);
[Xd5,Xv5,t5f] = GenTraj(Xfeedback(5),Xgoal(5),0,Tmax);
tall = [t1f t2f t3f t4f t5f]
Xd = [Xd1; Xd2; Xd3; Xd4; Xd5 ]
Xv = [Xv1; Xv2; Xv3; Xv4; Xv5 ]
%% ได้ Xti และ Xdti แต่ละเวลาออกมาจากนั้นแยกเข้าคำนวณ IVK และ IPK
%IPK ถ้าviapoints ห่างเกินอาจะทำให้เกิดการ singularity / แก้ inverse ไม่ได้
%แก้ได้โดยการทำให้ viapoint ถี่ขึ้น
n= size(Xd);
qbar=[]
for i =1:n(2)
    [qbari,shout] = IKnawhale2(Xd(1:3,i)',[0 Xd(4:5,i)'],gammabar,h1,h2,l1,l2,l3,DH);
    qbar(:,i)=qbari';
end
%% IVK
qvbar=[]
for i =1:n(2)
    [qvbari] = IVK(qbar(:,i),Xv(:,i));
    qvbar(:,i)=qvbari';
end
