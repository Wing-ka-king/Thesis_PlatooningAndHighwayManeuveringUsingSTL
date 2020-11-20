%12.0.3.50 : MAX
%12.0.5.5: JERRY

clc
clear all
close all
setenv('ROS_MASTER_URI','http://12.0.10.60:11311');
setenv('ROS_IP','12.0.10.60');
rosshutdown
pause(0.5);
rosinit('12.0.10.60',11311);

global positionJerry positionMax orientnJerry orientnMax time velJerry velMax
log_cmd  = zeros(1,9);
log_b = [0 0];

l_t = 40; %cm truck length
w_t = 40; %cm truck width
w_l = 52; %cm lane width

velParam.satVx = 70; %mms-1
velParam.satVy = 50; %mms-1
velParam.traffic_vel = 10; %mms-1
 
lane_poseMax = rossubscriber('/qualisys/MAX/pose');
lane_poseJerry = rossubscriber('/qualisys/Jerry/pose');

initialPoseMax = receive(lane_poseMax,5);
initialPoseJerry = receive(lane_poseJerry,5);
options = optimoptions('quadprog','Display','off');

hypParam1.a1 = -31.5/4;
hypParam1.b1 = 32;
hypParam1.a2 = -2.5/4;
hypParam1.b2 = 3.5;
hypParam1.a3 = -0.1;
hypParam1.b3 = -0.5;
hypParam1.c3 = -1.2;
hypParam1.d3 = 0.8;
hypParam1.a4 = 78;
hypParam1.b4 = 50;
hypParam1.k1 = 1;%7.5;
hypParam1.k2 = 2;%2.5;
hypParam1.k3 = 2;
hypParam1.ydes_a = initialPoseMax.Pose.Position.X;
ydes_b = 0.52;

hypParam2.a2 = -59.5/4;%-53/4;%-0.75;
hypParam2.b2 = 60;%54;%0.1;
hypParam2.a3 = -0.1;
hypParam2.b3 = -0.5;
hypParam2.c3 = -1.2;
hypParam2.d3 = 1;
hypParam2.a4 = 78;
hypParam2.b4 = 50;
hypParam2.k1 = 1;%7.5;
hypParam2.k2 = 0.5;%1;%2.5;
hypParam2.k3 = 0.8;
hypParam2.ydes_b = ydes_b;

[cmd_velJerry, velMessageJerry] = rospublisher('/JERRY/cmdvel', 'geometry_msgs/Twist');
[cmd_velMax, velMessageMax] = rospublisher('/MAX/cmdvel','geometry_msgs/Twist');

subs_velJerry = rossubscriber('/qualisys/Jerry/velocity',@callback_velJerry);
subs_velMax = rossubscriber('/qualisys/MAX/velocity',@callback_velMax);
subs_poseMax = rossubscriber('/qualisys/MAX/pose',@callback_poseMax);
subs_poseJerry = rossubscriber('/qualisys/Jerry/pose',@callback_poseJerry);
% subs_pwm = rossubscriber('/pwm_wheels',@callback_pwm);

velMessageJerry.Linear.X = 0;
velMessageJerry.Linear.Y = 0;
velMessageJerry.Angular.Z = 0;
velMessageMax.Linear.X = 0;
velMessageMax.Linear.Y = 0;
velMessageMax.Angular.Z = 0;

while isempty(time)
    pause(2)
    initial_time = length(time);
    relTime = time(end) - time(initial_time);
end
while (relTime <= 4)
    [uRob, uTraffic,b_final] = lane1_sml(relTime, positionMax(end,:), orientnMax(end,:),...
        positionJerry(end,:),orientnJerry(end,:),velParam, hypParam1,options);
    velMessageJerry.Linear.X = uTraffic(2)*10;
    velMessageJerry.Linear.Y = uTraffic(3)*10;
    velMessageMax.Linear.X = uRob(2)*10;
    velMessageMax.Linear.Y = uRob(3)*10;
    log_cmd = [log_cmd; uRob uTraffic relTime];
    log_b = [log_b; b_final relTime];
    send(cmd_velJerry, velMessageJerry)
    send(cmd_velMax, velMessageMax)
    pause(0.05)
    relTime = time(end) - time(initial_time);
end

disp("Attempting Lane Change")

t_ini = time(end);
relTime = time(end) - t_ini;
while ((abs(positionMax(end,2)-ydes_b) > 0.01) && relTime <= 4)
        relTime = time(end) - t_ini;
        [uRob, uTraffic,b_final] = lane2_sml(relTime, positionMax(end,:), orientnMax(end,:),...
            positionJerry(end,:),orientnJerry(end,:),velParam, hypParam2,options);
        velMessageJerry.Linear.X = uTraffic(2)*10;
        velMessageJerry.Linear.Y = uTraffic(1)*10;
         velMessageMax.Linear.X = uRob(2)*10;
        velMessageMax.Linear.Y = uRob(1)*10;
        log_cmd = [log_cmd; uRob uTraffic relTime];
        log_b = [log_b; b_final relTime];
        send(cmd_velJerry, velMessageJerry)
        send(cmd_velMax, velMessageMax)
        pause(0.05)
        
    if b_final < -0.6
        t_ini = time(end);
        relTime = time(end) - t_ini;
        while (relTime <= 1)
            [uRob, uTraffic] = laneAlt_sml(relTime, positionMax(end,:), orientnMax(end,:),...
                positionJerry(end,:),orientnJerry(end,:),velParam, hypParam1,options);
            velMessageJerry.Linear.X = uTraffic(2)*10;
            velMessageJerry.Linear.Y = uTraffic(3)*10;
            velMessageMax.Linear.X = uRob(2)*10;
            velMessageMax.Linear.Y = uRob(3)*10;
            log_cmd = [log_cmd; uRob uTraffic relTime];
            log_b = [log_b; 0 relTime];
            send(cmd_velJerry, velMessageJerry)
            send(cmd_velMax, velMessageMax)
            pause(0.05)
            relTime = time(end) - t_ini;
        end
        t_ini = time(end);
    end
end

fprintf("\nStopping and saving data...");

for n=1:50   
    velMessageJerry.Linear.X = 0;
    velMessageJerry.Linear.Y = 0;
    velMessageMax.Linear.X = 0;
    velMessageMax.Linear.Y = 0;
    send(cmd_velJerry, velMessageJerry)
    send(cmd_velMax, velMessageMax)
end

rosshutdown

absTime1 = 0;
absTime2 = 0;
for i=2:length(log_b)
    if log_b(i,2) == 0
        absTime1 = log_b(i-1,2) + absTime1;
        absTime2 = log_cmd(i-1,9) + absTime2;
    end
    log_b(i,3) = log_b(i,2) + absTime1;
    log_cmd(i,10) = log_cmd(i,9) + absTime2;
end

%%
close all
figure
% plot(positionJerry(:,1),positionJerry(:,2))
% hold on
% plot(positionMax(:,1),positionMax(:,2))
% legend("Traffic","Max")
% figure
plot(log_b(:,end), log_b(:,1))
title("Total Control Barrier Function")
xlabel("Time(s)")
figure
subplot(1,2,1), plot(log_cmd(:,end),log_cmd(:,2))
xlabel("Time(s)")
ylabel("Velocity in longitudinal direction(cm/s)")
title("u_1")
subplot(1,2,2), plot(log_cmd(:,end),log_cmd(:,1))
xlabel("Time(s)")
ylabel("Velocity in lateral direction(cm/s)")
title("u_2")

%%
function callback_poseJerry(~, message)
    global positionJerry orientnJerry time
    time = [time; (message.Header.Stamp.Sec + message.Header.Stamp.Nsec*1e-9)];
    positionJerry = [positionJerry ;message.Pose.Position.X message.Pose.Position.Y message.Pose.Position.Z];
    orientnJerry = [orientnJerry; message.Pose.Orientation.X message.Pose.Orientation.Y ...
        message.Pose.Orientation.Z message.Pose.Orientation.W];
end
function callback_poseMax(~, message)
    global positionMax orientnMax
    positionMax = [positionMax ;message.Pose.Position.X message.Pose.Position.Y message.Pose.Position.Z];
    orientnMax = [orientnMax; message.Pose.Orientation.X message.Pose.Orientation.Y ...
        message.Pose.Orientation.Z message.Pose.Orientation.W];
end

% function callback_pwm(~, message)
%     global pwm
%     pwm = [pwm; message.Data.'];
% end

function callback_velJerry(~, message)
    global velJerry
    velJerry = [velJerry; message.Twist.Linear.X message.Twist.Linear.Y message.Twist.Linear.Z];
end

function callback_velMax(~, message)
    global velMax
    velMax = [velMax; message.Twist.Linear.X message.Twist.Linear.Y message.Twist.Linear.Z];
end