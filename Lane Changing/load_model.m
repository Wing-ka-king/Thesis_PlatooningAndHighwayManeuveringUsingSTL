function [fx,gx] = load_model(x)

l_t = 8.2;
v = x(3);
psi = x(4);

fx = [v*cos(psi);
    v*sin(psi);
    0;
    0];
gx = [0 -v*sin(psi)/2;
    0 v*cos(psi)/2;
    1 0;
    0 v/l_t];

end