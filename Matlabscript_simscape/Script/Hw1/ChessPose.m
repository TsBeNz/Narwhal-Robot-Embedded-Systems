function [positionx,positiony] = ChessPose(X,N,theta,l,L)
% X คือ คอลัมภ์
% N คือ แถว
% l คือ ระยะจากเฟรมกลางหุ่นยนต์ไปถึงขอบล่างของกระดานหมากรุก
% L คือ ความยาวกระดานหมากรุก
% theta คือ มุมแกน y ของกระดานที่หมุนไปเทียบกับแกน y ของเฟรมหุ่นยนต์

positionx = sin(theta)*(((X*L)/8) - ((9*L)/16)) + (((N*L)/8) -((9*L)/16)*cos(theta)) + l + (L/2)
positiony = -cos(theta)*(((X*L)/8) - ((9*L)/16)) + (((N*L)/8) -((9*L)/16))*sin(theta)

end

