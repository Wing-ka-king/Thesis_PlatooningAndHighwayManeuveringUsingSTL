function [uRob,uTraffic] = laneAlt_sml(~,robPose, ~, trafficPose, trafficOrient, velParam,~,~)

% y1 = robPose(1);
% x1 = robPose(2);
% yi = trafficPose(1);
% xi = trafficPose(2);

% fprintf("The angle wrt x-axis %8.3f degrees\n ",rad2deg(atan2(robOrient(1),robOrient(2))));
% fprintf("The angle wrt y-axis %8.3f degrees\n",rad2deg(atan2(robOrient(2),robOrient(1))));

traffic_vel = velParam.traffic_vel;

uRob = zeros(1,4);

u2(1,2) = -traffic_vel;
u2(1,1) = 0;

uTraffic = quatmultiply(quatmultiply(trafficOrient, [u2 0 0]), quatinv(trafficOrient));
disp("Following Alt constraints")
end