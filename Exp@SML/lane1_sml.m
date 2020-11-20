function [uRob,uTraffic,b_final] = lane1_sml(t,robPose, robOrient, trafficPose, trafficOrient,velParam,hypParam,options)

y1 = robPose(1)*100;
x1 = robPose(2)*100;
yi = trafficPose(1)*100;
xi = trafficPose(2)*100;

% fprintf("The angle wrt x-axis %8.3f degrees\n ",rad2deg(atan2(robOrient(1),robOrient(2))));
% fprintf("The angle wrt y-axis %8.3f degrees\n",rad2deg(atan2(robOrient(2),robOrient(1))));

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
ydes_a = hypParam.ydes_a*100;

gamma = 1;

traffic_vel = velParam.traffic_vel;
sat_vx = velParam.satVx;
sat_vy = velParam.satVy;

% g1 = a1*t + b1;
% b_mu1 = k1*(g1^2 - (x1-30)^2);
% d1 = k1*(-2*(x1-30));
% t1 = k1*(2*g1*a1);

g2 = a2*t + b2;
% b_mu2 = k2*(g2^2 - (y1-ydes_a)^2);
% d2 = k2*(-2*(y1-ydes_a));
% t2 = k2*(2*g2*a2);
b_mu2 = k2*(g2 - abs(y1-ydes_a));
d2 = k2*(-(y1-ydes_a)/(abs(y1-ydes_a)+0.001));
t2 = k2*a2;

g3 = a3*exp(b3*t+c3)+d3;
b_col = k3*(((x1-xi)/a4)^2+((y1-yi)/(b4))^2 - g3);
d3x = k3*(2*(x1-xi)/(a4*a4));
d3y = k3*(2*(y1-yi)/(b4*b4));
t3 = k3*(2*(x1-xi)*(-traffic_vel)/(a4*a4) - b3*a3*exp(b3*t+c3));

g4 = a3*exp(b3*t+c3)+d3+2;
b_col2 = k3*((g4 - (x1-xi)/a4)^2 +((y1-yi)/(b4))^2);
d3x_2 = k3*(-2*(x1-xi)/(a4*a4));
d3y_2 = k3*(-2*(y1-yi)/(b4*b4));
t3_2 = k3*(-2*(x1-xi)*(-traffic_vel)/(a4*a4) + b3*a3*exp(b3*t+c3));

b_final = -log(exp(-b_mu2)+exp(-b_col));%+exp(-b_col2));

den =  -1/(exp(-b_mu2)+exp(-b_col));%+exp(-b_col2));

A = den*[exp(-b_col)*(-d3x);...
    exp(-b_mu2)*(-d2) + exp(-b_col)*(-d3y)];
b = den*(exp(-b_mu2)*(-t2) + exp(-b_col)*(-t3)) ...
    + gamma*b_final;


Q = [2 0;0 2];
g = [20; 0];
lb = [-sat_vx; -sat_vy];
ub = [0; sat_vy];

if b_final < 0
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f",t,b_col2,b_mu2,b_col,b_final)
    fprintf("\n %-8.3f %-8.3f %-8.3f %-8.3f \n",x1,y1,xi,yi)
end

if A == zeros(2,1)
    u = zeros(2,1);
    disp("Uncontrollable @ ")
    disp(t)
else
    [u,~,exitflag] = quadprog(Q,g,-transpose(A),b,[],[],lb,ub,[],options);
end

if exitflag~=1
    disp("No soln found \n")
    u = zeros(2,1);
end

uRob = 1*quatmultiply(quatmultiply(robOrient, [u(2,1) u(1,1) 0 0]),quatinv(robOrient));

u2(1,2) = -traffic_vel;
u2(1,1) = 0;

uTraffic = 1*quatmultiply(quatmultiply(trafficOrient, [u2 0 0]), quatinv(trafficOrient));

end