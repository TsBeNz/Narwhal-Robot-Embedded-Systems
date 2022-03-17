%% Thanks SANPOOMGOD
qk = [0;0;0;0;0];
qk1 = zeros(1,5);
T = 2.2*1000;
T2 = 4.4*1000;
T3 = 6*1000;
T4 = 10*1000;
m1 = zeros(4400,1);
m2 = zeros(4400,1);
m3 = zeros(4400,1);
m4 = zeros(4400,1);
m5 = zeros(4400,1);
m6 = zeros(4400,1);
Tm = zeros(1,4400);
% delta_p = [0.000;0.000;-0.000;0.0;-0.0;-0.01];
delta_p = [0.000;0.000;0.05];
delta_q = [0;0;0;0;0];
Tc = 0;
TT = (0.001:0.001:T);
for t = 0:T
    
    delta_q = IVK(qk,0,delta_p);
    qk1 = qk + delta_q
    m1(t+1) = qk1(1);
    m2(t+1) = qk1(2);
    m3(t+1) = qk1(3);
    m4(t+1) = qk1(4);
    m5(t+1) = qk1(5);
%     m6(t+1) = qk1(6);
    Tm(t+1) = Tc;
    Tc = Tc+0.001;
    qk = qk1;

end
% delta_p = [0.00;0.05;0.000];
% for t = 2200:T2
% 
%     
%     delta_q = IVK(qk,0,delta_p);
%     qk1 = qk + delta_q
%     m1(t+1) = qk1(1);
%     m2(t+1) = qk1(2);
%     m3(t+1) = qk1(3);
%     m4(t+1) = qk1(4);
%     m5(t+1) = qk1(5);
% %     m6(t+1) = qk1(6);
%     Tm(t+1) = Tc;
%     Tc = Tc+0.001;
%     qk = qk1;
% 
% end
% delta_p = [0.05;0.000;0.000];
% for t = 4400:T3
% 
%     
%     delta_q = IVK(qk,0,delta_p);
%     qk1 = qk + delta_q
%     m1(t+1) = qk1(1);
%     m2(t+1) = qk1(2);
%     m3(t+1) = qk1(3);
%     m4(t+1) = qk1(4);
%     m5(t+1) = qk1(5);
% %     m6(t+1) = qk1(6);
%     Tm(t+1) = Tc;
%     Tc = Tc+0.001;
%     qk = qk1;
% 
% end
% delta_p = [-0.05;0.000;0.000];
% for t = 6000:T4
% 
%     
%     delta_q = IVK(qk,0,delta_p);
%     qk1 = qk + delta_q
%     m1(t+1) = qk1(1);
%     m2(t+1) = qk1(2);
%     m3(t+1) = qk1(3);
%     m4(t+1) = qk1(4);
%     m5(t+1) = qk1(5);
% %     m6(t+1) = qk1(6);
%     Tm(t+1) = Tc;
%     Tc = Tc+0.001;
%     qk = qk1;
% 
% end
m1_t = timeseries(m1,Tm);
m2_t = timeseries(m2,Tm);
m3_t = timeseries(m3,Tm);
m4_t = timeseries(m4,Tm);
m5_t = timeseries(m5,Tm);
% m6_t = timeseries(m6,Tm);
%%
subplot(5,1,1)
plot(Tm,m1)
subplot(5,1,2)
plot(Tm,m2)
subplot(5,1,3)
plot(Tm,m3)
subplot(5,1,4)
plot(Tm,m4)
subplot(5,1,5)
plot(Tm,m5)