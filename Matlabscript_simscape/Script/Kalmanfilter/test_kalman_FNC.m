R = 10000;
Q = 0.001;
dt = 1;
x1 = 0;
x2 = 0;
p11 = 0.050;
p12 = 0.050;
p21 = 0.050;
p22 = 0.050;

n = 10000;
v_est = zeros(1,n);
vel = zeros(1,n);
pos_noise = zeros(1,n);
pos = zeros(1,n);

% x = -pi:0.01:pi;
% plot(x,sin(x)), grid on

for i = 100:n
    pos(i) = 2*pi*sin((i-100)*0.01);
    vel(i) = 2*pi*cos((i-100)*0.01);
    pos_noise(i) = pos(i) + 2*rand;
    [x1,x2,p11,p12,p21,p22] = kalmanFNC(R,Q,dt,pos_noise(i),x1,x2,p11,p12,p21,p22);
    v_est(i) = x2*100;
end

plot(v_est)
hold on
plot(vel)
