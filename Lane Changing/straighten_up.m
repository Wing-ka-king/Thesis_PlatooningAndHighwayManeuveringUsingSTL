function [u_tot,u,b_mu1,b_mu2,b_col,b_final,b1_deriv_test] = ...
    straighten_up(t,x,options,velParam,hypParam,t_ini)

x1 = x(1);
y1 = x(2);
v = x(3);
yaw = x(4);
xi = x(5);

a1 = hypParam.a1;
b1 = hypParam.b1;
a2 = hypParam.a2;
b2 = hypParam.b2;
k1 = hypParam.k1;
k2 = hypParam.k2;

gamma = 10;
ydes_b = 3.25;

[fx,gx] = load_model(x);

traffic_vel = velParam.traffic_vel;
sat_acc = velParam.satAcc;
sat_steer = velParam.satSteer;

g1 = a1*(t-t_ini) + b1;
b_mu1 = k1*(g1 - abs(yaw));
d1_yaw = k1*(-yaw/abs(yaw+eps));
t1_yaw = k1*(a1);

g2 = a2*(t-t_ini) + b2;
b_mu2 = k2*(g2^2 - (y1-ydes_b)^2);
d2 = k2*(-2*(y1-ydes_b));
t2 = k2*(2*g2*a2);

b_col = [0,0];
b_final = -log(exp(-b_mu1)+exp(-b_mu2));

global b_total_prev t_prev
if (t-t_ini)==0
    b_total_prev = 0;
    t_prev = 0;
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f",t,b_mu1,b_mu2,b_col,b_final)
    fprintf("\n x1=%-8.3f y1=%-8.3f yaw=%-8.3f xi=%-8.3f\n",x1, y1, yaw, xi)
end

% derivative test 
b_total_dot_1 = (b_final - b_total_prev)/(t-t_prev);
t_prev = t;
b_total_prev = b_final;

den = -1/(exp(-b_mu1)+exp(-b_mu2));

A = den*[0;
    exp(-b_mu2)*(-d2);
    0;
    exp(-b_mu1)*(-d1_yaw)];
b = den*(exp(-b_mu1)*(-t1_yaw)+exp(-b_mu2)*(-t2)) + gamma*b_final;

Q = [1 0;0 1];
g = [(v-10)/10; 0];
lb = [0; -sat_steer];
ub = [sat_acc; sat_steer];

if b_final < 0
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f",t,b_mu1,b_mu2,b_col,b_final)
    fprintf("\n x1=%-8.3f y1=%-8.3f yaw=%-8.3f xi=%-8.3f\n",x1, y1, yaw, xi)
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