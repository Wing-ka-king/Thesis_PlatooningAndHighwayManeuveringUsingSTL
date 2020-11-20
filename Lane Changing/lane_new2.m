function [u_tot,u,b_mu2,bCol,b_final,b1_deriv_test] = ...
    lane_new2(t,x,options,velParam,hypParam,t_ini)

x1 = x(1);
y1 = x(2);
xi = x(5);
yi = x(6);
xi_2 = x(7);
yi_2 = x(8);

a2 = hypParam.a2;
b2 = hypParam.b2;
a3 = hypParam.a3;
b3 = hypParam.b3;
c3 = hypParam.c3;
d3 = hypParam.d3;
a4 = hypParam.a4;
b4 = hypParam.b4;
k2 = hypParam.k2;
k3 = hypParam.k3;

gamma = 10;

ydes_b = 3.25;

[fx,gx] = load_model(x);

traffic_vel = velParam.traffic_vel;
sat_acc = velParam.satAcc;
sat_steer = velParam.satSteer;

g2 = a2*(t-t_ini) + b2;
b_mu2 = k2*(g2^2 - (y1-ydes_b)^2);
d2 = k2*(-2*(y1-ydes_b));
t2 = k2*(2*g2*a2);

g3 = a3*exp(b3*(t-t_ini)+c3)+d3;
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);
d3x = k3*(2*(x1-xi)/(a4*a4));
d3y = k3*(2*(y1-yi)/(b4*b4));
t3 = k3*(2*(x1-xi)*(-traffic_vel)/(a4*a4) - b3*a3*exp(b3*(t-t_ini)+c3));

a4 = a4 + 1.0;
k3 = 10*k3;
g4 = a3*exp(b3*t+c3)+(d3);
b_col2 = k3*(((x1-xi_2)/a4)^2+((y1-yi_2)/(b4))^2 - g4);
d4x = k3*(2*(x1-xi_2)/(a4*a4));
d4y = k3*(2*(y1-yi_2)/(b4*b4));
t4 = k3*(2*(x1-xi_2)*(-traffic_vel)/(a4*a4) - b3*a3*exp(b3*t+c3));

bCol = [b_col, b_col2];
b_final = -log(exp(-b_mu2)+exp(-b_col)+exp(-b_col2));

global b_total_prev t_prev
if (t-t_ini)==0
    b_total_prev = 0;
    t_prev = 0;
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f",t,b_mu2,b_col,b_final)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f\n",x1, y1, xi, yi)
end

% derivative test 
b_total_dot_1 = (b_final - b_total_prev)/(t-t_prev);
t_prev = t;
b_total_prev = b_final;

den =  -1/(exp(-b_mu2)+exp(-b_col)+exp(-b_col2));

A = den*[exp(-b_col)*(-d3x) + exp(-b_col2)*(-d4x);
    exp(-b_mu2)*(-d2) + exp(-b_col)*(-d3y) + exp(-b_col2)*(-d4y);
    0;
    0];
b = den*(exp(-b_mu2)*(-t2) + exp(-b_col)*(-t3) + exp(-b_col2)*(-t4)) + gamma*b_final;

Q = [1 0;0 10];
g = [-1; 0];
lb = [0; -sat_steer];
ub = [sat_acc; sat_steer];

if b_final < 0
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f",t,b_mu2,b_col,b_final)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f\n",x1, y1, xi, yi)
end

if A == zeros(4,1)
    u = zeros(2,1);
    disp("Uncontrollable @ ")
    disp(t)
else
    [u,~,exitflag] = quadprog(Q,g,-transpose(A)*gx,transpose(A)*fx+b,...
        [],[],[-inf;-sat_steer],[inf; sat_steer],[],options);
end

if exitflag ~= 1
    u = zeros(2,1);
end

% derivative test 
b_total_dot_2 = b - gamma*b_final + transpose(A)*(fx+gx*u);

b1_deriv_test = [b_total_dot_1,b_total_dot_2];

v_x1 = traffic_vel;
v_y1 = 0;

u_tot = 1.0*[fx + gx*u;v_x1;v_y1;v_x1;v_y1];
end