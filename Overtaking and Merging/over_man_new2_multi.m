function [u_tot,b_mu1,b_mu2,b_col,b_col_tp,b_final1,b1_deriv_test] = ...
    over_man_new2_multi(t,x,options,velParam,hypParam)

x1 = x(1);
y1 = x(2);
xi = x(3);
yi = x(4);
xp = (x(5)+x(7))/2;
yp = (x(6)+x(8))/2;

a1 = hypParam.a1;
b1 = hypParam.b1;
a2 = hypParam.a2;
b2 = hypParam.b2;
a3 = hypParam.a3;
b3 = hypParam.b3;
c3 = hypParam.c3;
d3 = hypParam.d3;
a4 = hypParam.a4;
b4 = hypParam.b4;
a5 = hypParam.a5;
b5 = hypParam.b5;
k1 = hypParam.k1;
k2 = hypParam.k2;
k3 = hypParam.k3;
k4 = hypParam.k4;
gamma = 10;

ydes_a = 0;

traffic_vel = velParam.traffic_vel;
sat_vx = velParam.satVx;
sat_vy = velParam.satVy;

g1 = a1*t + b1;
b_mu1 = k1*((x1-(xi+20)) - g1);
d1 = k1;
t1 = k1*(-traffic_vel - a1);

% b_mu1 = k1*(g1^2-(x1-xi-15)^2);
% d1 = k1*(-2*(x1-xi-15));
% t1 = k1*(2*g1*a1 - 2*(x1-xi-15)*(-traffic_vel));

g2 = a2*t + b2;
% b_mu2 = k2*(g2 - norm(y1-ydes_a));
% d2 = k2*(-(y1-ydes_a)/(norm(y1-ydes_a)+0.0001));
% t2 = k2*(a2);
b_mu2 = k2*(g2^2 - (y1-ydes_a)^2);
d2 = k2*(-2*(y1-ydes_a));
t2 = k2*(2*g2*a2);

g3 = a3*exp(b3*t+c3)+d3;
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);
% b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3^2);
d3x = k3*(2*(x1-xi)/(a4*a4));
d3y = k3*(2*(y1-yi)/(b4*b4));
t3 = k3*(2*(x1-xi)*(-traffic_vel)/(a4*a4) - b3*a3*exp(b3*t+c3));
% t3 = k3*(2*(x1-xi)*(-traffic_vel)/(a4*a4) - 2*g3*b3*a3*exp(b3*t+c3));

g4 = a3*exp(b3*t+c3)+d3;
b_col_tp = k4*(((x1-xp)/a5)^2+((y1-yp)/(b5))^2 - g4);
d4xi = -k4*(2*(x1-xp)/(a5*a5));
d4xp = -k4*(-2*(x1-xp)/(a5*a5));
d4yi = -k4*(2*(y1-yp)/(b5*b5));
d4yp = -k4*(-2*(y1-yp)/(b5*b5));
t4 = k4*(-b3*a3*exp(b3*t+c3));

% b_final1 = -log(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col)+exp(-b_col_tp));
b_final1 = -log(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col));

global b_total_prev t_prev
if t==4
    b_total_prev = [0; 0];
    t_prev = 0;
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f",t,b_mu1,b_mu2,b_col,b_col_tp,b_final1)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f \n",x1,y1,xi,xp)
end

% den = -1/(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col)+exp(-b_col_tp));
den = -1/(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col));


% A = den*[exp(-b_mu1)*(-d1) + exp(-b_col)*(-d3x) + exp(-b_col_tp)*(-d4xi);...
%     exp(-b_mu2)*(-d2) + exp(-b_col)*(-d3y) + exp(-b_col_tp)*(-d4yi)];
% 
% b = den*(exp(-b_mu1)*(-t1) + exp(-b_mu2)*(-t2) + exp(-b_col)*(-t3) + 0.5*exp(-b_col_tp)*(-t4)) ...
%     + gamma*b_final1;

A = den*[exp(-b_mu1)*(-d1) + exp(-b_col)*(-d3x), (1/den)*(d4xi);...
    exp(-b_mu2)*(-d2) + exp(-b_col)*(-d3y), (1/den)*(d4yi)];

b = [den*(exp(-b_mu1)*(-t1) + exp(-b_mu2)*(-t2) + exp(-b_col)*(-t3)) + gamma*b_final1;
     0.05*(t4 + gamma*2*b_col_tp)];

Q = [1 0; 0 1];
lb = [0; -sat_vy];
ub = [sat_vx; sat_vy];

if b_final1 < 0
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f",t,b_mu1,b_mu2,b_col,b_col_tp,b_final1)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f \n",x1,y1,xi,xp)
end

if A == zeros(2,1)
    u = [0;0];
    disp("Uncontrollable @ ")
    disp(t)
else
    u = quadprog(Q,zeros(2,0),-transpose(A),b,[],[],[0;-inf],[sat_vx;inf],[],options);
end

b_final2 = b_col_tp;
A_2 = -1*[d4xp;
    d4yp];
b_2 = 0.95*(t4 + gamma*2*b_final2);

Q_2 = [1 0; 0 1];

if A_2 == zeros(2,1)
    u_2 = [traffic_vel;0];
    disp("Uncontrollable @ ")
    disp(t)
else
    [u_2,~,exitflag] = quadprog(Q_2,zeros(2,0),-transpose(A_2),b_2,[],[],[traffic_vel;0],[inf;0],[],options);
end

if exitflag ~= 1
    u_2 = [traffic_vel;0];
end

% derivative test 
b_total_dot_1 = ([b_final1; b_final2] - b_total_prev)/(t-t_prev);
t_prev = t;
b_total_prev = [b_final1; b_final2];

% derivative test 
b_total_dot_2 = [b(1) - gamma*b_final1 + A(:,1).'*u; 
    1/0.95*b_2 - gamma*2*b_final2 + A_2.'*u_2];
    

% b1_deriv_test = [b1_deriv_test; [b_total_dot_1,b_total_dot_2]];
b1_deriv_test = [b_total_dot_1',b_total_dot_2'];


v_x1 = traffic_vel;
v_y1 = 0;

u_tot = 1.0*[u(1);u(2);v_x1;v_y1;u_2(1);u_2(2);u_2(1);u_2(2)];

end