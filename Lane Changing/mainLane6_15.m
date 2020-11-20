%%
% New model : X = [x y v psi]
% Changes : changed parameter b2 for straighten_up
%         : changed parameter a2 for lane_alt
%         : altered deceleration value in optimization of lane_alt
%         : increased tolerance for rear vehicle
%         : increased weightage of lane change constraint in hypParam2

% No gap case : traffic_vel 10 and 12.5 behave in a similar manner



%% Main file for Lane Changing Scenario

clear all
clc

l_t = 8.2; %m truck length
w_t = 2.5; %m truck width
w_l = 3.25; %m lane width
ydes_a = 0;
ydes_b = 3.25;

velParam.satAcc = 4; %ms-2
velParam.satSteer = inf; %rad/s
velParam.traffic_vel = 10; %ms-1

y0 = [0 0 10 0 0 3.25 -45.1 3.25];
hypParam1.a1 = -31.5/4;
hypParam1.b1 = 32;
hypParam1.a2 = -3.0/3;
hypParam1.b2 = 3;
hypParam1.a3 = -0.1;
hypParam1.b3 = -0.5;
hypParam1.c3 = -1.2;
hypParam1.d3 = 0.8;
hypParam1.a4 = 16;
hypParam1.b4 = 3.1;
hypParam1.k1 = 10;%7.5;
hypParam1.k2 = 1;%2.5;
hypParam1.k3 = 5;

options_qp = optimoptions('quadprog','Display','off');
atol_ode = 1e-12;
rtol_ode = 1e-12;
options_1 = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);

t_start = 0;
t_final1 = 2;

global b_stop t_ini hypParam2
b_stop = 0.5;
tic

[tout, yout] = ode45(@(t,y)lane_new1(t,y,options_qp,velParam,hypParam1),...
    [t_start t_final1], y0, options_1);
m = length(tout);
u = zeros(m,2);
b5 = zeros(m,1);
b6 = zeros(m,1);
b_c = zeros(m,2);
b_tot = zeros(m,1);
deriv_test = zeros(m,2);
for i = 1:m
    [~,u(i,:),b6(i),b_c(i,:),b_tot(i),deriv_test(i,:)] = ...
        lane_new1(tout(i),yout(i,:),options_qp,velParam,hypParam1);
    if mod(i,1e3) == 0
        disp(tout(i))
    end
end

hypParam2.a1 = -31.5/4;
hypParam2.b1 = 63.5;
hypParam2.a2 = -3.49/4;
hypParam2.b2 = 3.5;
hypParam2.a3 = -0.1;
hypParam2.b3 = -0.5;
hypParam2.c3 = -1.2;
hypParam2.d3 = 1.05;
hypParam2.a4 = 16;
hypParam2.b4 = 3.2;
hypParam2.k1 = 1;%7.5;
hypParam2.k2 = 5;%2.5;
hypParam2.k3 = 20;

hypParamAlt.a1 = -4.5;%0.25;
hypParamAlt.b1 = 11;%-1;
hypParamAlt.c1 = 1;
hypParamAlt.d1 = 49;
hypParamAlt.a2 = -3.24/2;
hypParamAlt.b2 = 1.5;
hypParamAlt.a3 = -0.1;
hypParamAlt.b3 = -0.5;
hypParamAlt.c3 = -1.2;
hypParamAlt.d3 = 0.8;
hypParamAlt.a4 = 16;
hypParamAlt.b4 = 3.2;
hypParamAlt.k1 = 10;%7.5;
hypParamAlt.k2 = 10;%2.5;
hypParamAlt.k3 = 2;

options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode,'Events',@alternateEvents);
ranTimes = 0;
while ((abs(yout(end,2)-ydes_b)>0.2))% && yout(end,1)<120))% || yout(end,4)>0)
    disp(ranTimes)
    t_ini = tout(end)
    [tout_t, yout_t,~,~,~] = ode45(@(t,y)lane_new2(t,y,options_qp,velParam,hypParam2,t_ini),...
        [tout(end) tout(end)+4], yout(end,:), options);
    n = length(tout_t);
    u_t = zeros(n,2);
    b5_t = zeros(n,1);
    b6_t = zeros(n,1);
    b_c_t = zeros(n,2);
    b_tot_t = zeros(n,1);
    deriv_test_t = zeros(n,2);
    for i = 1:n
        [~,u_t(i,:),b6_t(i),b_c_t(i,:),b_tot_t(i),deriv_test_t(i,:)] = ...
            lane_new2(tout_t(i),yout_t(i,:),options_qp,velParam,hypParam2,t_ini);
        if mod(i,1e3) == 0
            disp(tout_t(i))
        end
    end

    tout = [tout; tout_t];
    yout = [yout; yout_t];
    u = [u; u_t];
    b5 = [b5; b5_t];
    b6 = [b6; b6_t];
    b_c = [b_c; b_c_t];
    b_tot = [b_tot; b_tot_t];
    deriv_test = [deriv_test; deriv_test_t];

    if yout(end,2)< 2
        t_ini = tout(end)
        [tout_t, yout_t] = ode45(@(t,y)lane_alt(t,y,options_qp,velParam,hypParamAlt,t_ini),...
            [tout(end) tout(end)+1], yout(end,:), options);
        n = length(tout_t);
        u_t = zeros(n,2);
        b5_t = zeros(n,1);
        b6_t = zeros(n,1);
        b_c_t = zeros(n,2);
        b_tot_t = zeros(n,1);
        deriv_test_t = zeros(n,2);
        for i = 1:n
            [~,u_t(i,:),b5_t(i),b6_t(i),b_c_t(i,:),b_tot_t(i),deriv_test_t(i,:)] = ...
                lane_alt(tout_t(i),yout_t(i,:),options_qp,velParam,hypParamAlt,t_ini);
            if mod(i,1e3) == 0
                disp(tout_t(i))
            end
        end

        tout = [tout; tout_t];
        yout = [yout; yout_t];
        u = [u; u_t];
        b5 = [b5; b5_t];
        b6 = [b6; b6_t];
        b_c = [b_c; b_c_t];
        b_tot = [b_tot; b_tot_t];
        deriv_test = [deriv_test; deriv_test_t];

        ranTimes = ranTimes + 1;
        disp(yout(end,:))

    else
        ranTimes = ranTimes + 1;
        disp(yout(end,:))
        continue
    end
    
end

hypParam3.a1 = -0.009/2;
hypParam3.b1 = 0.2;
hypParam3.a2 = -0.1/2;
hypParam3.b2 = 0.2;
hypParam3.k1 = 100;
hypParam3.k2 = 50;

[tout_t, yout_t] = ode45(@(t,y)straighten_up(t,y,options_qp,velParam,hypParam3,tout(end)),...
    [tout(end) tout(end)+2.5], yout(end,:), options_1);
disp("Aligning the car")
m = length(tout_t);
u_t = zeros(m,2);
b5_t = zeros(m,1);
b6_t = zeros(m,1);
b_c_t = zeros(m,2);
b_tot_t = zeros(m,1);
deriv_test_t = zeros(m,2);
for i = 1:m
    [~,u_t(i,:),b5_t(i),b6_t(i),b_c_t(i,:),b_tot_t(i),deriv_test_t(i,:)] = ...
        straighten_up(tout_t(i),yout_t(i,:),options_qp,velParam,hypParam3, tout(end));
    if mod(i,1e3) == 0
        disp(tout_t(i))
    end
end

tout = [tout; tout_t];
yout = [yout; yout_t];
u = [u; u_t];
b5 = [b5; b5_t];
b6 = [b6; b6_t];
b_c = [b_c; b_c_t];
b_tot = [b_tot; b_tot_t];
deriv_test = [deriv_test; deriv_test_t];

toc

%% For movie

tout(:,2) = [0;diff(tout(:,1))];
for i=1:length(tout)
if tout(i,2) == 0
t_ini_val = tout(i,1);
end
tout(i,3) = tout(i,1)-t_ini_val;
end

traj_1 = [yout(:,1),yout(:,2),yout(:,3),yout(:,4)];
traj_2 = [yout(:,5),yout(:,6)];
traj_3 = [yout(:,7),yout(:,8)];

% edges = linspace(0,yout(end,1),501);
% label_traj = discretize(yout(:,1),edges);
edges = linspace(0,tout(end,1),501);
label_traj = discretize(tout(:,1),edges);

label_traj = [label_traj;0]-[0;label_traj];
label_traj(isnan(label_traj)) = 0;
label_traj = logical(label_traj);
label_traj(end) = [];

traj_1 = traj_1(label_traj,:);
traj_2 = traj_2(label_traj,:);
traj_3 = traj_3(label_traj,:);
plot_tout = tout(label_traj,3);

u2 = u(:,2);
u2(u(:,2)>=0.5) = 0.5;
u2(u(:,2)<=-0.5) = -0.5;


%% Plots

close all
tic

% subplot(1,3,1), plot(yout(:,2),yout(:,1))
% hold on
% subplot(1,3,1), plot(yout(:,6),yout(:,5))
% xlabel("Width")
% ylabel("Length")
% title("Trajectory")
% subplot(1,3,2), plot(tout, u(:,1))
% xlabel("Time")
% title("Acceleration u(1)")
% subplot(1,3,3), plot(tout, u(:,2))
% xlabel("Time")
% title("Steering Angle u(2)")

subplot(1,2,1), plot(tout(1:end-300,1), u(1:end-300,1))
ylabel("Acceleration (m/s^2)")
xlabel("Time(s)")
title("u_1")
subplot(1,2,2), plot(tout(1:end-300,1), u2(1:end-300))
xlabel("Time(s)")
ylabel("Steering Angle (rad)")
title("u_2")


% figure
% subplot(2,2,1), plot(tout(:,1), b5(:))
% title("Lane 1\_x track CBF")
% subplot(2,2,2), plot(tout(:,1), b6(:))
% title("Lane 1\_y track CBF")
% subplot(2,2,3), plot(tout(:,1), b_tot(:))
% title("Total Control barrier Function")
% subplot(2,2,4), plot(tout(:,1), b_c(:))
% title("Collision BF")
figure
plot(tout(:,1), b_tot(:))
title("Total Control barrier Function")
xlabel("Time(s)")


figure,
subplot(1,4,1), plot(tout(:,1),yout(:,2)), title("Lateral position")
ylabel("y (m)")
xlabel("Time(s)")
hold on
% subplot(1,2,1), plot(tout, yout(:,6))
subplot(1,4,1), plot(tout(:,1), yout(:,6))
% legend("Final Trajectory","Without Collision Constraints","Traffic")
subplot(1,4,2), plot(tout(:,1),yout(:,1)), title("Longitudinal position")
ylabel("x (m)")
xlabel("Time(s)")
hold on
% subplot(1,2,2), plot(tout, yout(:,5))
subplot(1,4,2), plot(tout(:,1), yout(:,5))
% legend("Final Trajectory","Without Collision Constraints","Traffic")
subplot(1,4,3), plot(tout(:,1), yout(:,3)), title("Velocity")
ylabel("v (m/s)")
xlabel("Time(s)")
subplot(1,4,4), plot(tout(:,1), yout(:,4)), title("Yaw Angle")
ylabel("\psi (rad)")
xlabel("Time(s)")

toc


%% Visual representation of the Simulation

close all

t_eval = linspace(-pi,pi,20);

xplot3 = @(t1) 16*cos(t1);
yplot3 = @(t1) 3.2*sin(t1);
xeval3 = feval(xplot3,t_eval);
yeval3 = feval(yplot3,t_eval);

clear F
F(size(traj_1,1)) = struct('cdata',[],'colormap',[]);

for t = 1:size(traj_1,1)    

    X_t1 = traj_2(t,1);
    Y_t1 = traj_2(t,2);
    X_t2 = traj_3(t,1);
    Y_t2 = traj_3(t,2);
    
    X_ego = traj_1(t,1);
    Y_ego = traj_1(t,2);
    theta_leading = traj_1(t,4);

    hold off
    leading_frontleft  = [X_t1+l_t/2,Y_t1+w_t/2];
    leading_frontright = [X_t1+l_t/2,Y_t1-w_t/2];
    leading_rearleft   = [X_t1-l_t/2,Y_t1+w_t/2];
    leading_rearright  = [X_t1-l_t/2,Y_t1-w_t/2];
    
    leading_frontleft2  = [X_t2+l_t/2,Y_t2+w_t/2];
    leading_frontright2 = [X_t2+l_t/2,Y_t2-w_t/2];
    leading_rearleft2   = [X_t2-l_t/2,Y_t2+w_t/2];
    leading_rearright2  = [X_t2-l_t/2,Y_t2-w_t/2];
    
    ego_frontleft  = [X_ego+l_t/2*cos(theta_leading)-w_t/2*sin(theta_leading),...
        Y_ego+l_t/2*sin(theta_leading)+w_t/2*cos(theta_leading)];
    ego_frontright = [X_ego+l_t/2*cos(theta_leading)+w_t/2*sin(theta_leading),...
        Y_ego+l_t/2*sin(theta_leading)-w_t/2*cos(theta_leading)];
    ego_rearleft   = [X_ego-l_t/2*cos(theta_leading)-w_t/2*sin(theta_leading),...
        Y_ego-l_t/2*sin(theta_leading)+w_t/2*cos(theta_leading)];
    ego_rearright  = [X_ego-l_t/2*cos(theta_leading)+w_t/2*sin(theta_leading),...
        Y_ego-l_t/2*sin(theta_leading)-w_t/2*cos(theta_leading)];
    
    xplot3 = xeval3 + X_t1;
    yplot3 = yeval3 + Y_t1;
    xplot4 = xeval3 + X_t2;
    yplot4 = yeval3 + Y_t2;
    
    time_text = string(plot_tout(t));
    toprint = [time_text, "s"];
    
    plot([leading_frontleft(1),leading_frontright(1),leading_rearright(1),leading_rearleft(1),leading_frontleft(1)],...
         [leading_frontleft(2),leading_frontright(2),leading_rearright(2),leading_rearleft(2),leading_frontleft(2)],'b')

    hold on
    plot([leading_frontleft2(1),leading_frontright2(1),leading_rearright2(1),leading_rearleft2(1),leading_frontleft2(1)],...
         [leading_frontleft2(2),leading_frontright2(2),leading_rearright2(2),leading_rearleft2(2),leading_frontleft2(2)],'b')

    plot([ego_frontleft(1),ego_frontright(1),ego_rearright(1),ego_rearleft(1),ego_frontleft(1)],...
         [ego_frontleft(2),ego_frontright(2),ego_rearright(2),ego_rearleft(2),ego_frontleft(2)],'r')

    
    plot([-100,1000],[-1.625,-1.625],'k')
    plot([-100,1000],[8.125,8.125],'k')    
    plot([-100,1000],[1.625,1.625],'--g')
    plot([-100,1000],[4.875,4.875],'--g')
    
    text(X_t1-5,-5,join(toprint))
    plot(xplot3,yplot3,'-r')
    plot(xplot4,yplot4,'-r')
%     plot(X_ego,Y_ego,'*')
%     line([50 50],[-1.625 1.625],'LineStyle',':','Color','k','Linewidth',2)
    
    axis([X_ego-40,X_ego+80, -25, 30])
    F(t) = getframe(gcf);
    F(t) = getframe(gca);
    
    pause(0.001)
    
end
%%

close all
movie(F,1,24)
%%

function [value,isterminal,direction] = alternateEvents(t,y)
x1 = y(1);
y1 = y(2);
xi = y(3);
yi = y(4);

global hypParam2
a2 = hypParam2.a2;
b2 = hypParam2.b2;
a3 = hypParam2.a3;
b3 = hypParam2.b3;
c3 = hypParam2.c3;
d3 = hypParam2.d3;
a4 = hypParam2.a4;
b4 = hypParam2.b4;

k2 = hypParam2.k2;%2.5;
k3 = hypParam2.k3;

ydes_b = 3.25;
global b_stop
global t_ini

g2 = a2*(t-t_ini) + b2;
b_mu2 = k2*(g2^2 - (y1-ydes_b)^2);
g3 = a3*exp(b3*(t-t_ini)+c3)+d3;
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);
g4 = a3*exp(b3*(t-t_ini)+c3)+d3+5;
b_col2 = k3*((g4 - (x1-xi)/a4)^2+((y1-yi)/(b4))^2);

b_final = -log(exp(-b_mu2)+exp(-b_col)+exp(-b_col2));
value = b_final-b_stop;
isterminal = 1;
direction = 1;

end