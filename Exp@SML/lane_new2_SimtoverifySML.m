function [u_tot,b_mu1,b_mu2,b_col,b_final,b1_deriv_test] = ...
    lane_new2_SimtoverifySML(t_org,x,options,velParam,hypParam,t_ini)

global b_stop
x1 = x(1);
y1 = x(2);
xi = x(3);
yi = x(4);

a1 = 0;
b1 = 0;
a2 = hypParam.a2;
b2 = hypParam.b2;
a3 = hypParam.a3;
b3 = hypParam.b3;
c3 = hypParam.c3;
d3 = hypParam.d3;
a4 = hypParam.a4;
b4 = hypParam.b4;
k1 = hypParam.k1;
k2 = hypParam.k2;
k3 = hypParam.k3;
gamma = 1;%20;

ydes_b = 52.0;

traffic_vel = velParam.traffic_vel;
sat_vx = velParam.satVx;
sat_vy = velParam.satVy;

t = t_org-t_ini;

g1 = a1*t + b1;
b_mu1 = k1*(g1^2 - (x1-60)^2);
d1 = k1*(-2*(x1-60));
t1 = k1*(2*g1*a1);
% b_mu1 = k1*(g1 - abs(x1-60));
% d1 = k1*(-(x1-60)/abs(x1-60));
% t1 = k1*(a1);

g2 = a2*t + b2;
% b_mu2 = k2*(g2^2 - (y1-ydes_b)^2);
% d2 = k2*(-2*(y1-ydes_b));
% t2 = k2*(2*g2*a2);
b_mu2 = k2*(g2 - abs(y1-ydes_b));
d2 = k2*(-(y1-ydes_b)/abs(y1-ydes_b));
t2 = k2*(a2);

g3 = a3*exp(b3*t+c3)+d3;
% b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3^2);
% d3x = k3*(2*(x1-xi)/(a4*a4));
% d3y = k3*(2*(y1-yi)/(b4*b4));
% t3 = k3*(2*(x1-xi)*(-traffic_vel)/(a4*a4) - 2*g3*b3*a3*exp(b3*t+c3));
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);
d3x = k3*(2*(x1-xi)/(a4*a4));
d3y = k3*(2*(y1-yi)/(b4*b4));
t3 = k3*(2*(x1-xi)*(-traffic_vel)/(a4*a4) - b3*a3*exp(b3*t+c3));

g4 = a3*exp(b3*t+c3)+d3+2;
b_col2 = k3*((g4 - (x1-xi)/a4)^2 -((y1-yi)/(b4))^2);
d3x_2 = k3*(-2*(x1-xi)/(a4*a4));
d3y_2 = k3*(-2*(y1-yi)/(b4*b4));
t3_2 = k3*(-2*(x1-xi)*(-traffic_vel)/(a4*a4) + b3*a3*exp(b3*t+c3));

% b_final = -log(exp(-b_col2)+exp(-b_mu2)+exp(-b_col));
b_final = -log(exp(-b_mu2)+exp(-b_col));


global b_total_prev t_prev
if t==0
    b_total_prev = 0;
    t_prev = 0;
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f",t_org,b_final,b_mu2,b_col)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f\n",x1,y1,xi,yi)
end

% derivative test 
b_total_dot_1 = (b_final - b_total_prev)/(t-t_prev);
t_prev = t;
b_total_prev = b_final;

% den =  -1/(exp(-b_mu2)+exp(-b_col)+exp(-b_col2));
den =  -1/(exp(-b_mu2)+exp(-b_col));

% A = den*[exp(-b_col)*(-d3x)+exp(-b_col2)*(-d3x_2);...
%     exp(-b_mu2)*(-d2) + exp(-b_col)*(-d3y)+exp(-b_col2)*(-d3y_2)];
% b = den*(exp(-b_mu2)*(-t2) + exp(-b_col)*(-t3) + exp(-b_col2)*(-t3_2)) ...
%     + gamma*b_final;

A = den*[exp(-b_col)*(-d3x);
    exp(-b_mu2)*(-d2) + exp(-b_col)*(-d3y)];
b = den*(exp(-b_mu2)*(-t2) + exp(-b_col)*(-t3)) ...
    + gamma*b_final;


% Q = [1 0; 0 1];
Q = [2 0;0 1];
g = [-20.0; 0];
lb = [0; -sat_vy];
ub = [sat_vx; sat_vy];

if b_final < b_stop
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f",t_org,b_final,b_mu2,b_col)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f\n",x1,y1,xi,yi)
end


if A == zeros(2,1)
    u = zeros(2,1);
    disp("Uncontrollable @ ")
    disp(t)
else
    [u,~,exitflag] = quadprog(Q,g,-transpose(A),b,[],[],lb,ub,[],options);
end

if exitflag ~= 1
    u = [0;0];
end

% derivative test 
b_total_dot_2 = b - gamma*b_final + A.'*u;

% b1_deriv_test = [b1_deriv_test; [b_total_dot_1,b_total_dot_2]];
b1_deriv_test = [b_total_dot_1,b_total_dot_2];

v_x1 = traffic_vel;
v_y1 = 0;

u_tot = 1.0*[u(1);u(2);v_x1;v_y1];

end