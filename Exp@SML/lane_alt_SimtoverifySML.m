function [u_tot,b_mu1,b_mu2,b_col,b_final,b1_deriv_test] = ...
    lane_alt_SimtoverifySML(t,x,options,velParam,hypParam,t_ini)

x1 = x(1);
y1 = x(2);
xi = x(3);
yi = x(4);

a1 = hypParam.a1;
b1 = hypParam.b1;
c1 = hypParam.c1;
d1 = hypParam.d1;
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

gamma = 10;
ydes_a= 0;

traffic_vel = velParam.traffic_vel;
sat_vx = velParam.satVx;
sat_vy = velParam.satVy;

% g1 = a1*(t-t_ini) + b1;
b_mu1 = 0;
% d1v = k1*(-2*(v-1));
% t1 = k1*(2*g1*a1);

g2 = a2*(t-t_ini) + b2;
b_mu2 = k2*(g2^2 - (y1-ydes_a)^2);
b_mu2 = 0;
d2 = k2*(-2*(y1-ydes_a));
t2 = k2*(2*g2*a2);

g3 = a3*exp(b3*(t-t_ini)+c3)+d3;
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);
d3x = k3*(2*(x1-xi)/(a4*a4));
d3y = k3*(2*(y1-yi)/(b4*b4));
t3 = k3*(2*(x1-xi)*(-traffic_vel)/(a4*a4) - b3*a3*exp(b3*(t-t_ini)+c3));

% b_final = -log(exp(-b_mu1)+exp(-b_mu2));%+exp(-b_col));
b_final = b_col;

global b_total_prev t_prev
if (t-t_ini)==0
    b_total_prev = 0;
    t_prev = 0;
    fprintf("\n %-8.3f %-8.3f",t, b_mu2)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f\n",x1, y1, xi, yi)
%     b1_deriv_test = b1_deriv_test_ini;
end

% derivative test 
b_total_dot_1 = (b_final - b_total_prev)/(t-t_prev);
t_prev = t;
b_total_prev = b_final;


% den = -1/(exp(-b_mu1)+exp(-b_mu2)+exp(-b_col));
% den = -1/(exp(-b_mu1)+exp(-b_mu2));

A = [0;    d2];
b = t2 + gamma*b_final;

Q = [1 0;0 1];
g = [-0.5*traffic_vel; 0];
lb = [0; -sat_vy];
ub = [sat_vx; sat_vy];

if b_final < 0
    fprintf("\n %-8.3f %-8.3f",t,b_mu2)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f\n",x1, y1, xi, yi)
end

% if A == zeros(2,1)
%     u = zeros(2,1);
%     disp("Uncontrollable @ ")
%     disp(t)
% else
%     [u,~,exitflag] = quadprog(Q,g,-transpose(A),b,[],[],[0;-sat_vy],[sat_vx; sat_vy],[],options);
% end
% 
% if exitflag ~= 1
%     u = zeros(2,1);
% end

u(1,1) = 0;
u(2,1) = 0;

% derivative test 
b_total_dot_2 = b - gamma*b_final + transpose(A)*(u);

% b1_deriv_test = [b1_deriv_test; [b_total_dot_1,b_total_dot_2]];
b1_deriv_test = [b_total_dot_1,b_total_dot_2];

v_x1 = traffic_vel;
v_y1 = 0;

u_tot = 1.0*[u(1);u(2);v_x1;v_y1];

end