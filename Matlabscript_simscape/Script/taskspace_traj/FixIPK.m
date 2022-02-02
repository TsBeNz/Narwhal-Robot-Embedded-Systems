% link length Update
h1= 275.99; % lasted
h2= 380;
l1= 20.01;
l2= 380;
l3= 268.23;
%%%  lasted update
% h1 = 295.89;
% h2 =380.00;
% l1 =20;
% l2 = 380.00;
% l3= 268.23;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TaskVar = [ (l2+l1) 0 (h1+h2-l3) -pi pi]% position(x,y,z) and pitch ,yaw
TaskVar = [ 400 50 50 -pi pi]
%(65.4+416) 0 ((180+310-150)) 0 0               (20+380) 0 ((251+380-285))
P = TaskVar(1:3)'
rbar = [0 TaskVar(4:5)] 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DH =[0   0   h1   0;
    l1 pi/2  0  pi/2;
    h2  0    0  -pi/2;
    l2  0    0    0;
    0  pi/2  0    0];

% gammabar = [1 1 1];
gammabar = [1 1 1 1];
[qbar] = IKnawhale2(P,rbar,gammabar,h1,h2,l1,l2,l3,DH)
[H1,H2,H3,H4,H5,He] = FKnawhale(qbar,l3,DH)
karn= qbar