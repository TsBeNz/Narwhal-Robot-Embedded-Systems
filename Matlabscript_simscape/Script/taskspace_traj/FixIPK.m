% link length Update
h1= 275.99; % lasted
h2= 380;
l1= 20;
l2= 380;
l3= 268.23;
%%%  lasted update
% h1 = 295.89;
% h2 =380.00;
% l1 =20;
% l2 = 380.00;
% l3= 268.23;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TaskVar = [ (l2+l1) 0 (h1+h2-l3) -pi pi]% position(x,y,z) and pitch ,yaw
% TaskVar = [ (154.68) 0 (330.43) -pi pi]
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
% [qbar] = IKnarwhale3(P,rbar,gammabar,h1,h2,l1,l2,l3)
test = [0;
    0.8900;
    1.2696;
   -2.1596;
    3.1416]
[H1,H2,H3,H4,H5,He] = FKnawhale(qbar,l3,DH)  
% f = eul2rotm([],'ZYZ')
karn= qbar
%% data to simscape
q1sim = timeseries(q1VSt,linspace(0,Tmax,numel(q1VSt)));
q2sim = timeseries(q2VSt,linspace(0,Tmax,numel(q2VSt)));
q3sim = timeseries(q3VSt,linspace(0,Tmax,numel(q3VSt)));
q4sim = timeseries(q4VSt,linspace(0,Tmax,numel(q4VSt)));
q5sim = timeseries(q5VSt,linspace(0,Tmax,numel(q5VSt)));
