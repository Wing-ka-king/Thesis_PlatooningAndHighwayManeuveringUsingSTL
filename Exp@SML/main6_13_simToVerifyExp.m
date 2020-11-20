clear all
clc

% format int32

l_t = 40.0; %mm truck length
w_t = 40.0; %mm truck width
w_l = 52.0; %mm lane width

velParam.satVx = 70.0; %mms-1
velParam.satVy = 70.0; %mms-1
%avg speed = 25cm/s
velParam.traffic_vel = 25.0; % 35; %25 %mms-1

ydes_a = 0;
ydes_b = 52.0;

global hypParam2 b_stop t_ini
b_stop = 0;

y0 = [0 0 0 52.0];% 22.4 0 31.6 0];
hypParam1.a1 = -31.5/4;
hypParam1.b1 = 32.0;
hypParam1.a2 = -34/3.9;
hypParam1.b2 = 35;
hypParam1.a3 = -0.1;
hypParam1.b3 = -0.5;
hypParam1.c3 = -1.2;
hypParam1.d3 = 0.8;
hypParam1.a4 = 78.0;
hypParam1.b4 = 50.0;
hypParam1.k1 = 1;%7.5;
hypParam1.k2 = 5;%2.5;
hypParam1.k3 = 1;

options_qp = optimoptions('quadprog','Display','off');
atol_ode = 1e-2;
rtol_ode = 1e-2;
options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);

t_start = 0;
t_final1 = 3;
t_final2 = 7;
t_final3 = 12;
t_total = 0;
x = y0;

tic

[tout, yout] = ode45(@(t,y)lane_new1_SImtoverifySML(t,y,options_qp,velParam,hypParam1),...
    [t_start t_final1], y0, options);
m = length(tout);
u = zeros(m,4);
b5 = zeros(m,1);
b6 = zeros(m,1);
b_c = zeros(m,1);
b_tot = zeros(m,1);
deriv_test = zeros(m,2);
for i = 1:m
    [u(i,:),b5(i),b6(i),b_c(i),b_tot(i),deriv_test(i,:)] = ...
        lane_new1_SImtoverifySML(tout(i),yout(i,:),options_qp,velParam,hypParam1);
    if mod(i,1e3) == 0
        disp(tout(i))
    end
end

hypParam2.a2 = -59.5/4;%-53/4;%-0.75;
hypParam2.b2 = 60;%0.1;
hypParam2.a3 = -0.1;
hypParam2.b3 = -0.5;
hypParam2.c3 = -1.2;
hypParam2.d3 = 0.8;
hypParam2.a4 = 78;
hypParam2.b4 = 50;
hypParam2.k1 = 1;%7.5;
hypParam2.k2 = 10;%2.5;
hypParam2.k3 = 20;

hypParamAlt.a1 = -4.5;%0.25;
hypParamAlt.b1 = 11;%-1;
hypParamAlt.c1 = 1;
hypParamAlt.d1 = 49;
hypParamAlt.a2 = -53.0/4;
hypParamAlt.b2 = 54.0;
hypParamAlt.a3 = -0.1;
hypParamAlt.b3 = -0.5;
hypParamAlt.c3 = -1.2;
hypParamAlt.d3 = 0.75;
hypParamAlt.a4 = 78.0;
hypParamAlt.b4 = 50.0;
hypParamAlt.k1 = 1;%7.5;
hypParamAlt.k2 = 1;%2.5;
hypParamAlt.k3 = 1;

options2 = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode,'Events',@alternateEvents);
ranTimes = 0;
while ((abs(yout(end,2)-ydes_b)>1))% && yout(end,1)<120))% || yout(end,4)>0)
    disp(ranTimes)
    t_ini = tout(end)
   
    [tout_t, yout_t,~,~,~] = ode45(@(t,y)lane_new2_SimtoverifySML(t,y,options_qp,velParam,hypParam2,t_ini),...
        [tout(end) tout(end)+4], yout(end,:), options2);
    fprintf("Exiting lane2 specs")
    n = length(tout_t);
    u_t = zeros(n,4);
    b5_t = zeros(n,1);
    b6_t = zeros(n,1);
    b_c_t = zeros(n,1);
    b_tot_t = zeros(n,1);
    deriv_test_t = zeros(n,2);
    for i = 1:n
        [u_t(i,:),b5_t(i),b6_t(i),b_c_t(i),b_tot_t(i),deriv_test_t(i,:)] = ...
            lane_new2_SimtoverifySML(tout_t(i),yout_t(i,:),options_qp,velParam,hypParam2,t_ini);
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
%     hypParam2.a2 = -abs(yout(end,2)-ydes_b)/4;
%     hypParam2.b2 = (ydes_b-yout(end,2))+0.1;
    b_stop = 1e-12;
    
    if (yout(end,1)< 100)
        t_ini = tout(end)
        [tout_t, yout_t] = ode45(@(t,y)lane_alt_SimtoverifySML(t,y,options_qp,velParam,hypParamAlt,t_ini),...
            [tout(end) tout(end)+2], yout(end,:), options);
        n = length(tout_t);
        u_t = zeros(n,4);
        b5_t = zeros(n,1);
        b6_t = zeros(n,1);
        b_c_t = zeros(n,1);
        b_tot_t = zeros(n,1);
        deriv_test_t = zeros(n,2);
        for i = 1:n
            [u_t(i,:),b5_t(i),b6_t(i),b_c_t(i),b_tot_t(i),deriv_test_t(i,:)] = ...
                lane_alt_SimtoverifySML(tout_t(i),yout_t(i,:),options_qp,velParam,hypParamAlt,t_ini);
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

toc

%%

traj_1 = [yout(:,1),yout(:,2)];
traj_2 = [yout(:,3),yout(:,4)];

% edges = linspace(0,yout(end,1),201);
% label_traj = discretize(traj_1(:,1),edges);
edges = linspace(0,tout(end),501);
label_traj = discretize(tout,edges);
label_traj = [label_traj;0]-[0;label_traj];
label_traj(isnan(label_traj)) = 0;
label_traj = logical(label_traj);
label_traj(end) = [];

traj_1 = traj_1(label_traj,:);
traj_2 = traj_2(label_traj,:);

%%

close all
tic

subplot(1,3,1), plot(yout(:,2),yout(:,1))
hold on
subplot(1,3,1), plot(yout(:,4),yout(:,3))
xlabel("Width")
ylabel("Length")
title("Trajectory")
legend("Ego","Traffic")
subplot(1,3,2), plot(tout, u(:,1))
xlabel("Time")
ylabel("v_x")
legend("Ego Vehicle","Platoon")
subplot(1,3,3), plot(tout, u(:,2))
xlabel("Time")
ylabel("v_y")
legend("Ego Vehicle","Platoon")

figure
subplot(2,2,1), plot(tout, b5(:))
title("Lane 1\_x track CBF")
subplot(2,2,2), plot(tout, b6(:))
title("Lane 1\_y track CBF")
subplot(2,2,3), plot(tout, b_tot(:))
title("Total Control barrier Function")
subplot(2,2,4), plot(tout, b_c(:))
title("Collision BF")

figure,
subplot(1,2,1), plot(tout,yout(:,2)), title("Y direction as a function of time")
hold on
subplot(1,2,1), plot(tout, yout(:,4))
subplot(1,2,2), plot(tout,yout(:,1)), title("X direction as a function of time")
hold on
subplot(1,2,2), plot(tout, yout(:,3))

toc


%%

close all

theta_leading = 0;
t_eval = linspace(-pi,pi,20);
% xplot = @(t1) 16*cos(t1);
% yplot = @(t1) 8*sin(t1);
% xeval = feval(xplot,t_eval);
% yeval = feval(yplot,t_eval);
% 
% xplot2 = @(t1) 13.2*cos(t1);
% yplot2 = @(t1) 10*sin(t1);
% xeval2 = feval(xplot2,t_eval);
% yeval2 = feval(yplot2,t_eval);

xplot3 = @(t1) 78*cos(t1);
yplot3 = @(t1) 50*sin(t1);
xeval3 = feval(xplot3,t_eval);
yeval3 = feval(yplot3,t_eval);

clear F
F(size(traj_1,1)) = struct('cdata',[],'colormap',[]);
for t = 1:size(traj_1,1)    

    X_t1 = traj_2(t,1);
    Y_t1 = traj_2(t,2);
    
    X_ego = traj_1(t,1);
    Y_ego = traj_1(t,2);
        
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
    
    xplot3 = xeval3 + X_t1;
    yplot3 = yeval3 + Y_t1;
    
    plot([leading_frontleft(1),leading_frontright(1),leading_rearright(1),leading_rearleft(1),leading_frontleft(1)],...
         [leading_frontleft(2),leading_frontright(2),leading_rearright(2),leading_rearleft(2),leading_frontleft(2)],'b')

    hold on
    plot([ego_frontleft(1),ego_frontright(1),ego_rearright(1),ego_rearleft(1),ego_frontleft(1)],...
         [ego_frontleft(2),ego_frontright(2),ego_rearright(2),ego_rearleft(2),ego_frontleft(2)],'r')
    
    plot([0,1000],[-26,-26],'k')
    plot([0,1000],[130,130],'k')    
    plot([0,1000],[26,26],'--g')
    plot([0,1000],[78,78],'--g')
    
    plot(xplot3,yplot3)
    plot(X_ego,Y_ego,'*')
    
    axis([X_ego-150,X_ego+200, -200, 250])
    F(t) = getframe(gcf);
    F(t) = getframe(gca);
    
    pause(0.001)
    
end
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

ydes_b = 52;
global b_stop
global t_ini

g2 = a2*(t-t_ini) + b2;
b_mu2 = k2*(g2^2 - (y1-ydes_b)^2);
g3 = a3*exp(b3*(t-t_ini)+c3)+d3;
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);

b_final = -log(exp(-b_mu2)+exp(-b_col));%+exp(-b_col2));
value = b_final-b_stop;
isterminal = 1;
direction = 0;

end