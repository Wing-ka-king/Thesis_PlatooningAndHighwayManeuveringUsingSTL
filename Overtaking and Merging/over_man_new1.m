function [u_tot,b_mu1,b_mu2,b_col,b_final,b1_deriv_test] = ...
    over_man_new1(t,x,options,velParam,hypParam)

x1 = x(1);
y1 = x(2);
xi = x(3);
yi = x(4);

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
k1 = hypParam.k1;
k2 = hypParam.k2;
k3 = hypParam.k3;
gamma = 10;%20;

ydes_b = 3.25;

traffic_vel = velParam.traffic_vel;
sat_vx = velParam.satVx;
sat_vy = velParam.satVy;

g1 = a1*t + b1;
% b_mu1 = k1*(g1^2 - (x1-xi)^2);
% d1 = k1*(-2*(x1-xi));
% t1 = k1*(2*g1*a1 - 2*(x1-xi)*(-traffic_vel));
b_mu1 = k1*(g1 - abs(x1-xi));
d1 = k1*(-(x1-xi)/abs(x1-xi));
t1 = k1*(a1 - (x1-xi)*(-traffic_vel)/abs(x1-xi));

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

b_final = -log(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col));
% b_final = min([b_mu1, b_mu2, b_col]);

global b_total_prev t_prev
if t==0
    b_total_prev = 0;
    t_prev = 0;
%     b1_deriv_test = b1_deriv_test_ini;
end

% derivative test 
b_total_dot_1 = (b_final - b_total_prev)/(t-t_prev);
t_prev = t;
b_total_prev = b_final;

den = -1/(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col));

A = den*[exp(-b_mu1)*(-d1) + exp(-b_col)*(-d3x);...
    exp(-b_mu2)*(-d2) + exp(-b_col)*(-d3y)];

b = den*(exp(-b_mu1)*(-t1) + exp(-b_mu2)*(-t2) + exp(-b_col)*(-t3)) ...
    + gamma*b_final;

Q = [1 0; 0 1];
lb = [0; -sat_vy];
ub = [sat_vx; sat_vy];

if b_final < 0
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f",t,b_mu1,b_mu2,b_col)
end

if A == zeros(2,1)
    u = zeros(2,1);
    disp("Uncontrollable @ ")
    disp(t)
else
    [u,~,exitflag] = quadprog(Q,zeros(2,0),-transpose(A),b,[],[],[0,-inf],[sat_vx,inf],[],options);
end

if exitflag~=1
    u = [0;0];
end

% derivative test 
b_total_dot_2 = b - gamma*b_final + A.'*u;

% b1_deriv_test = [b1_deriv_test; [b_total_dot_1,b_total_dot_2]];
b1_deriv_test = [b_total_dot_1, 0, b_total_dot_2, 0];

% plot(b_total_dot_1,b_total_dot_2,'*')
% legend("Numerical","Calculated")
% drawnow

v_x1 = traffic_vel;
v_y1 = 0;

u_p(1) = traffic_vel;
u_p(2) = 0;

u_tot = 1.0*[u(1);u(2);v_x1;v_y1;u_p(1);u_p(2);u_p(1);u_p(2)];
% g = @(x)[u(1);u(2);v_x1;v_y1;u_p(1);u_p(2);u_p(1);u_p(2)];
% x_n = rk4(g,dt,x(1:8).');
% x_n = x + dt*u_tot.';

end