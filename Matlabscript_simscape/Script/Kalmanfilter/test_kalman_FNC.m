R = 50000;
Q = 0.0001;
dt = 1;
x1 = 0;
x2 = 0;
p11 = 0.050;
p12 = 0.050;
p21 = 0.050;
p22 = 0.050;

n = 10000;
t = linspace(0,10,n);

pos = 2*pi*sin(t);
vel = 2*pi*cos(t);
acc = -2*pi*sin(t);
pos_noise = zeros(1,n);
v_est = zeros(1,n);

for i = 1:n
    pos_noise(i) = pos(i) + (- 1 + 2*rand); 
    [x1,x2,p11,p12,p21,p22] = kalmanFNC(R,Q,dt,pos_noise(i),x1,x2,p11,p12,p21,p22);
    v_est(i) = x2*1000;
end

plot(v_est)
hold on
plot(vel)

