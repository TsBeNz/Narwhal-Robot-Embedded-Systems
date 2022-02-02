syms theta_k w_k Q R dt x1 x2 p11 p12 p21 p22 real
%state space model
A = [1 dt;
     0  1];
G = [dt^2/2 ;
       dt] 
C = [1 0];
% Q = [1]; 
% R =[1];
x_est = [x1;x2];
p_est = [p11 p12;
         p21 p22];
y_fb = [1 0]*[theta_k;w_k];
%% 
x_pred = A*x_est 
p_pred = A*p_est*A'+ G*Q*G'
y_pred = C*x_pred
y_re = y_fb - y_pred
s = C * p_pred * C' + R
k = p_pred * C' /s
x_est = simplify(x_pred + k*y_re)
p_est = simplify((eye(2)-k*C)*p_pred)



%SADKARN
%sadpoomGOD
%FUCK U MAN 