clc,clear,close all;

path = "D:\成大\碩一\新訓\我的\9_MyRobot\CAD";

robot = MyRobot(path);

%% 
% EulerAngle = [];
% Position = [];
% for i = 1 : 360
%     EulerAngle = [EulerAngle; 0, 0, pi;];
%     Position = [Position; 30 + 5*cos(i*pi/180), 5*sin(i*pi/180), 30;];
% end
% 
% theta = Inverse_Kinematic(EulerAngle, Position);
% thetadeg = rad2deg(theta);
% p = [];
% pic_num = 1;
% for i = 1 : 20 : 360
%     figure(1)
%     robot.Draw(thetadeg(i, :));
% 
%     drawnow;
%     F = getframe(gcf);
%     I = frame2im(F);
%     [I,map]=rgb2ind(I,256);
%     
%     if pic_num == 1
%         imwrite(I,map,'test.gif','gif','Loopcount',inf,'DelayTime',0.2);
%     else
%         imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.2);
%     end
%     
%     pic_num = pic_num + 1;
%     hold off
%     pause(0.001)
% end

%% 
% robot.Link_RT([0, 90, 0], [50, 0, 90], 6)
% robot.temp_Draw(6, [255, 0, 0])

Angle = [0, 0, 0, 0, 0, 0];
robot.Draw(Angle)

