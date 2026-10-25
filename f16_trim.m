function [x0, u0, info] = f16_trim(V, h, P)
% Trim for steady, straight and level flight at speed V (ft/s), altitude h (ft).
% Unknowns: [alpha, throttle, elevator]. Uses fminsearch (no extra toolbox).

z0   = [0.04; 0.12; 0.01];
opts = optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',20000,'MaxIter',20000);
z    = fminsearch(@(z) trim_cost(z,V,h,P), z0, opts);
z    = fminsearch(@(z) trim_cost(z,V,h,P), z,  opts);   % restart to polish

[x0, u0] = build_state(z, V, h);
xd = f16_dynamics(x0, u0, P);
info.residual = norm(xd([1 3 8]));
info.alpha_deg = rad2deg(z(1));
info.throttle  = z(2);
info.elevator_deg = rad2deg(z(3));
end

function J = trim_cost(z, V, h, P)
[x, u] = build_state(z, V, h);
xd = f16_dynamics(x, u, P);
J = xd(1)^2 + xd(3)^2 + 100*xd(8)^2;     % udot, wdot, qdot -> 0
end

function [x, u] = build_state(z, V, h)
alpha = z(1); thr = z(2); de = z(3);
% theta = alpha  ->  flight path angle = 0 (level flight)
x = [V*cos(alpha); 0; V*sin(alpha); 0; alpha; 0; 0; 0; 0; 0; 0; h; 100*thr];
u = [thr; de; 0; 0];
end