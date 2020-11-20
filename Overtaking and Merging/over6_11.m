% known issues : saturated input not working - UPDATE:WORKING NOW
%              : More tuning required - TUNED
%              : The decentralised control for platoon requires additional
%              weight multiplied with its state-derivatives
%              : Change in state derivatives for de-centralized mode?

%%

clear all
clc

% format int32

l_t = 8.2; %m truck length
w_t = 2.5; %m truck width
w_l = 3.25; %m lane width

velParam.satVx = 20; %ms-1
velParam.satVy = 8; %ms-1
velParam.traffic_vel = 12.5; %ms-1

global hypParam2 b_stop
b_stop = 0.5;

y0 = [0 1 16 0 23.4+16 0 32.6+16 0];
hypParam1.a1 = -16.02/4;
hypParam1.b1 = 17;
hypParam1.a2 = -2.9/4;
hypParam1.b2 = 3;
hypParam1.a3 = -0.1;
hypParam1.b3 = -0.5;
hypParam1.c3 = -1.2;
hypParam1.d3 = 1;
hypParam1.a4 = 16;
hypParam1.b4 = 3.1;
hypParam1.k1 = 10;%7.5;
hypParam1.k2 = 100;%2.5;
hypParam1.k3 = 20;

options_qp = optimoptions('quadprog','Display','off');
atol_ode = 1e-12;
rtol_ode = 1e-12;
options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);

t_start = 0;
t_final1 = 4;
t_final2 = 8;
t_final3 = 12;
t_total = 0;
x = y0;

tic

[tout, yout] = ode45(@(t,y)over_man_new1(t,y,options_qp,velParam,hypParam1),...
    [t_start t_final1], y0, options);
m = length(tout);
u = zeros(m,8);
b5 = zeros(m,1);
b6 = zeros(m,1);
b_c = zeros(m,1);
b_tot = zeros(m,1);
b_c_tp = zeros(m,1);
deriv_test = zeros(m,4);
for i = 1:m
    [u(i,:),b5(i),b6(i),b_c(i),b_tot(i),deriv_test(i,:)] = ...
        over_man_new1(tout(i),yout(i,:),options_qp,velParam,hypParam1);
    if mod(i,1e3) == 0
        disp(tout(i))
    end
end

hypParam2.a1 = 21.2/4;%-15/4;%8.5/4;
hypParam2.b1 = -43;%31;%-10;
hypParam2.a2 = -3.6/2;%-3.45/4;
hypParam2.b2 = 14.5;%7.0;
hypParam2.a3 = -0.1;
hypParam2.b3 = -0.5;
hypParam2.c3 = -1.2;
hypParam2.d3 = 1;
hypParam2.a4 = 16;
hypParam2.b4 = 3.1;
hypParam2.a5 = 16;
hypParam2.b5 = 8;
hypParam2.k1 = 1;%7.5;z
hypParam2.k2 = 10;%2.5;
hypParam2.k3 = 10;
hypParam2.k4 = 1;

options2 = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode,'Events',@alternateEvents);
ranTimes = 0;
while yout(end,1) < yout(end,3)+15

    [tout_t, yout_t] = ode45(@(t,y)over_man_new2_multi(t,y,options_qp,velParam,hypParam2),...
        [tout(end) tout(end)+4], yout(end,:), options2);
    n = length(tout_t);
    u_t = zeros(n,8);
    b5_t = zeros(n,1);
    b6_t = zeros(n,1);
    b_c_t = zeros(n,1);
    b_col_tp_t = zeros(n,1);
    b_tot_t = zeros(n,1);
    deriv_test_t = zeros(n,4);

    for i = 1:n
        [u_t(i,:),b5_t(i),b6_t(i),b_c_t(i),b_col_tp_t(i),b_tot_t(i),deriv_test_t(i,:)] = ...
            over_man_new2_multi(tout_t(i),yout_t(i,:),options_qp,velParam,hypParam2);
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
    b_c_tp = [b_c_tp; b_col_tp_t];
    b_tot = [b_tot; b_tot_t];
    deriv_test = [deriv_test; deriv_test_t];
    
    disp(ranTimes)
    ranTimes = ranTimes +1;
    
end

hypParam3.a1 = -2.9/2;%-15/4;%8.5/4;
hypParam3.b1 = 14.6;%31;%-10;
hypParam3.a2 = -0.4/2;%-3.45/4;
hypParam3.b2 = 2.1;%7.0;
hypParam3.a3 = -0.1;
hypParam3.b3 = -0.5;
hypParam3.c3 = -1.2;
hypParam3.d3 = 1;
hypParam3.a4 = 16;
hypParam3.b4 = 3.1;
hypParam3.a5 = 13.2;
hypParam3.b5 = 10;
hypParam3.k1 = 1;%7.5;z
hypParam3.k2 = 1;%2.5;
hypParam3.k3 = 10;
hypParam3.k4 = 1;

atol_ode = 1e-8;
rtol_ode = 1e-8;
options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);

[tout_t, yout_t] = ode45(@(t,y)merging(t,y,options_qp,velParam,hypParam3),...
    [tout(end) tout(end)+2], yout(end,:), options);
n = length(tout_t);
u_t = zeros(n,8);
b5_t = zeros(n,1);
b6_t = zeros(n,1);
b_c_t = zeros(n,1);
b_col_tp_t = zeros(n,1);
b_tot_t = zeros(n,1);
deriv_test_t = zeros(n,4);

for i = 1:n
    [u_t(i,:),b5_t(i),b6_t(i),b_c_t(i),b_col_tp_t(i),b_tot_t(i),deriv_test_t(i,:)] = ...
        merging(tout_t(i),yout_t(i,:),options_qp,velParam,hypParam3);
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
b_c_tp = [b_c_tp; b_col_tp_t];
b_tot = [b_tot; b_tot_t];
deriv_test = [deriv_test; deriv_test_t];
    
toc
%%

tout(:,2) = [0;diff(tout(:,1))];
for i=1:length(tout)
if tout(i,2) == 0
t_ini_val = tout(i,1);
end
tout(i,3) = tout(i,1)-t_ini_val;
end

traj_1 = [yout(:,1),yout(:,2)];
traj_2 = [yout(:,3),yout(:,4)];
traj_3 = [yout(:,5),yout(:,6)];
traj_4 = [yout(:,7),yout(:,8)];

edges = linspace(0,yout(end,1),201);
label_traj = discretize(traj_1(:,1),edges);
label_traj = [label_traj;0]-[0;label_traj];
label_traj(isnan(label_traj)) = 0;
label_traj = logical(label_traj);
label_traj(end) = [];

traj_1 = traj_1(label_traj,:);
traj_2 = traj_2(label_traj,:);
traj_3 = traj_3(label_traj,:);
traj_4 = traj_4(label_traj,:);
plot_tout = tout(label_traj,3);


%%

close all
tic


subplot(1,2,1), plot(tout(:,1),yout(:,1))
hold on
subplot(1,2,1), plot(tout(:,1),yout(:,5))
subplot(1,2,1), plot(tout(:,1),yout(:,3))
xlabel("X")
ylabel("Time")
subplot(1,2,2), plot(tout(:,1),yout(:,2))
hold on
subplot(1,2,2), plot(tout(:,1),yout(:,6))
subplot(1,2,2), plot(tout(:,1),yout(:,4))
xlabel("Y")
ylabel("Time")
title("Trajectory")

figure
subplot(1,2,1), plot(tout(1:end-300,1), u(1:end-300,1))
ylabel("Velocity in lateral direction(m/s)")
xlabel("Time(s)")
title("u_1")
hold on
subplot(1,2,1), plot(tout(1:end-300,1), u(1:end-300,7))
legend("Ego Vehicle","Platoon",'location','best')

subplot(1,2,2), plot(tout(1:end-300,1), u(1:end-300,2))
xlabel("Time(s)")
ylabel("Velocity in longitudinal direction(cm/s)")
title("u_2")
hold on
subplot(1,2,2), plot(tout(1:end-300,1), u(1:end-300,8))
legend("Ego Vehicle","Platoon",'location','best')

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
title("Total Control barrier Functions")
hold on
plot(tout(:,1), b_c_tp(:))
xlabel("Time(s)")
legend("CBF for Ego Vehicle","CBF for platoon")


% figure,
% subplot(1,2,1), plot(tout(:,1), yout(:,2)), title("Lateral position")
% ylabel("y (m)")
% xlabel("Time(s)")
% hold on
% subplot(1,2,1), plot(tout(:,1), yout(:,6))
% subplot(1,2,2), plot(tout(:,1), yout(:,1)), title("Logitudinal position")
% ylabel("x (m)")
% xlabel("Time(s)")
% hold on
% subplot(1,2,2), plot(tout(:,1), yout(:,5))

toc

%%

close all

theta_leading = 0;
t_eval = linspace(-pi,pi,20);
xplot = @(t1) 16*cos(t1);
yplot = @(t1) 8*sin(t1);
xeval = feval(xplot,t_eval);
yeval = feval(yplot,t_eval);

% xplot2 = @(t1) 13.2*cos(t1);
% yplot2 = @(t1) 10*sin(t1);
% xeval2 = feval(xplot2,t_eval);
% yeval2 = feval(yplot2,t_eval);

xplot3 = @(t1) 16*cos(t1);
yplot3 = @(t1) 3.1*sin(t1);
xeval3 = feval(xplot3,t_eval);
yeval3 = feval(yplot3,t_eval);

clear F
F(size(traj_1,1)) = struct('cdata',[],'colormap',[]);
for t = 1:size(traj_1,1)    

    X_t1 = traj_2(t,1);
    Y_t1 = traj_2(t,2);
    
    X_ego = traj_1(t,1);
    Y_ego = traj_1(t,2);
    
    X_p1 = traj_3(t,1);
    Y_p1 = traj_3(t,2);
    
    X_p2 = traj_4(t,1);
    Y_p2 = traj_4(t,2);
    
    hold off
    leading_frontleft  = [X_t1+l_t/2*cos(theta_leading)-w_t/2*sin(theta_leading),...
        Y_t1+l_t/2*sin(theta_leading)+w_t/2*cos(theta_leading)];
    leading_frontright = [X_t1+l_t/2*cos(theta_leading)+w_t/2*sin(theta_leading),...
        Y_t1+l_t/2*sin(theta_leading)-w_t/2*cos(theta_leading)];
    leading_rearleft   = [X_t1-l_t/2*cos(theta_leading)-w_t/2*sin(theta_leading),...
        Y_t1-l_t/2*sin(theta_leading)+w_t/2*cos(theta_leading)];
    leading_rearright  = [X_t1-l_t/2*cos(theta_leading)+w_t/2*sin(theta_leading),...
        Y_t1-l_t/2*sin(theta_leading)-w_t/2*cos(theta_leading)];    
    
    ego_frontleft  = [X_ego+l_t/2,Y_ego+w_t/2];
    ego_frontright = [X_ego+l_t/2,Y_ego-w_t/2];
    ego_rearleft   = [X_ego-l_t/2,Y_ego+w_t/2];
    ego_rearright  = [X_ego-l_t/2,Y_ego-w_t/2];
    
    
    p1_frontleft = [X_p1+l_t/2,Y_p1+w_t/2];
    p1_frontright = [X_p1+l_t/2,Y_p1-w_t/2];
    p1_rearleft   = [X_p1-l_t/2,Y_p1+w_t/2];
    p1_rearright  = [X_p1-l_t/2,Y_p1-w_t/2];
    
    p2_frontleft = [X_p2+l_t/2,Y_p2+w_t/2];
    p2_frontright = [X_p2+l_t/2,Y_p2-w_t/2];
    p2_rearleft   = [X_p2-l_t/2,Y_p2+w_t/2];
    p2_rearright  = [X_p2-l_t/2,Y_p2-w_t/2];
    
    xplot = xeval + (X_p1+X_p2)/2;
    yplot = yeval + (Y_p1+Y_p2)/2;
%     xplot2 = xeval2 + (X_p1+X_p2)/2;
%     yplot2 = yeval2 + (Y_p1+Y_p2)/2;
    xplot3 = xeval3 + X_t1;
    yplot3 = yeval3 + Y_t1;
    
    time_text = string(plot_tout(t));
    toprint = [time_text, "s"];
    
    plot([leading_frontleft(1),leading_frontright(1),leading_rearright(1),leading_rearleft(1),leading_frontleft(1)],...
         [leading_frontleft(2),leading_frontright(2),leading_rearright(2),leading_rearleft(2),leading_frontleft(2)],'b')

    hold on
    plot([ego_frontleft(1),ego_frontright(1),ego_rearright(1),ego_rearleft(1),ego_frontleft(1)],...
         [ego_frontleft(2),ego_frontright(2),ego_rearright(2),ego_rearleft(2),ego_frontleft(2)],'r')
    plot([p1_frontleft(1),p1_frontright(1),p1_rearright(1),p1_rearleft(1),p1_frontleft(1)],...
         [p1_frontleft(2),p1_frontright(2),p1_rearright(2),p1_rearleft(2),p1_frontleft(2)],'r')
    plot([p2_frontleft(1),p2_frontright(1),p2_rearright(1),p2_rearleft(1),p2_frontleft(1)],...
         [p2_frontleft(2),p2_frontright(2),p2_rearright(2),p2_rearleft(2),p2_frontleft(2)],'r')

    
    plot([-100,1000],[-1.625,-1.625],'k')
    plot([-100,1000],[8.125,8.125],'k')    
    plot([-100,1000],[1.625,1.625],'--g')
    plot([-100,1000],[4.875,4.875],'--g')
    
    text(X_t1-5,-5,join(toprint))
    plot(xplot,yplot)
%     plot(xplot2, yplot2)
    plot(xplot3,yplot3)
%     plot(X_ego,Y_ego,'*')
    
    axis([X_ego-40,X_ego+80, -25, 30])
    F(t) = getframe(gcf);
    F(t) = getframe(gca);
    
    pause(0.05)
    
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
xp = (y(5)+y(7))/2;
yp = (y(6)+y(8))/2;

global hypParam2
a1 = hypParam2.a1;
b1 = hypParam2.b1;
a2 = hypParam2.a2;
b2 = hypParam2.b2;
a3 = hypParam2.a3;
b3 = hypParam2.b3;
c3 = hypParam2.c3;
d3 = hypParam2.d3;
a4 = hypParam2.a4;
b4 = hypParam2.b4;
a5 = hypParam2.a5;
b5 = hypParam2.b5;
k1 = hypParam2.k1;
k2 = hypParam2.k2;
k3 = hypParam2.k3;
k4 = hypParam2.k4;

ydes_a = 0;
global b_stop

g1 = a1*t + b1;
% b_mu1 = k1*((x1-xi) - g1);
b_mu1 = k1*(g1^2-(x1-xi-15)^2);

g2 = a2*t + b2;
b_mu2 = k2*(g2 - norm(y1-ydes_a));
% b_mu2 = k2*(g2^2 - (y1-ydes_a)^2);

g3 = a3*exp(b3*t+c3)+d3;
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);

g4 = a3*exp(b3*t+c3)+d3;
b_col_tp = k4*(((x1-xp)/a5)^2+((y1-yp)/(b5))^2 - g4);

b_final = -log(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col)+exp(-b_col_tp));

value = b_final-b_stop;
isterminal = 1;
direction = 1;

end