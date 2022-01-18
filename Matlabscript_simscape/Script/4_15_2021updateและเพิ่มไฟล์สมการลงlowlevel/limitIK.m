% define variables
clear
c =200*sqrt(2);%เส้นทะแยงุม
length = 0 :c/10:c;
robot2chessboard = 450;
rbar = [0 -pi pi];
% h1= 180;
% h2= 310;
% l1= 65.4;
% l2= 416;
% l3= 150;
h1= 251;
h2= 380;
l1= 20;
l2= 380;
l3= 235;
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];
gammabar = [1 1 1 1];
% calculation
qmax = zeros(1,5)
qmin = zeros(1,5)
failcount = 0
now =0
for i = 1: 1: 10 % จำนวนวง 
    status = i 
    for j = 1:1:360 % มุม
       for k = 1:2:30  % ความสูง แกน z  30 cm.
            x = (length(i)*cos((j*pi)/180)) + robot2chessboard;
            y = length(i)*sin((j*pi)/180);
            z = k*10;
            P =[x;y;z]
%             plot3(x,y,z,'.')

            [qbar,shout] = IKnawhale2(P,rbar,gammabar,h1,h2,l1,l2,l3,DH);
            for m = 1:5
                if qbar(m) > qmax(m)
                    qmax(m) = qbar(m);
                elseif qbar(m) < qmin(m)
                    qmin(m) = qbar(m);
                end
            end
            if shout == 1
                failcount = failcount + 1
            end
            now = now+1

%             hold on
       end
    end 
end
%% 
max =qmax *(180/pi)
min =qmin *(180/pi)