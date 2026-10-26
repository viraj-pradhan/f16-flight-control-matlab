function D = f16_design(P)
% Designs every controller used in steps 5 and 6 at the 500 ft/s, sea-level
% trim point. Returns a struct D with the trim, linear model and all gains.
%
%   Pitch SAS        LQR on [w q]                      (step 2)
%   Lateral dampers  yaw damper with washout + roll damper (step 3)
%   PID autopilot    cascaded loops (altitude, heading, airspeed)
%   LQI autopilot    LQR with integral action (altitude, heading)
%
% Needs Control System Toolbox (lqr).

V0 = 500;  h0 = 0;
[x0,u0] = f16_trim(V0,h0,P);
[A,B]   = numjac_f16(x0,u0,P);
D.V0 = V0;  D.x0 = x0;  D.u0 = u0;  D.A = A;  D.B = B;

% ---- pitch SAS (step 2) ----
sp = [3 8];
Q  = diag([1/(V0*deg2rad(5))^2, 1/deg2rad(30)^2]);
R  = 1/deg2rad(10)^2;
K  = lqr(A(sp,sp), B(sp,2), Q, R);
D.Kw = K(1);  D.Kq = K(2);

% ---- lateral dampers (step 3): roll damper fixed, yaw gain by fminbnd ----
D.Kp = 0.10;  D.Tw = 1.0;
lat = [2 4 7 9];                         % v phi p r
Al  = A(lat,lat);  Bl = B(lat,[3 4]);    % inputs: aileron, rudder
D.Kr = fminbnd(@(k) -dutch_damping(k,Al,Bl,D.Kp,D.Tw), 0, 3);

% ---- PID autopilot gains (hand tuned) ----
D.G = struct('Kh',0.2,'Kc',0.002,'Kci',0.003,'Kth',2.2,'Kthi',0.3, ...
             'Kpsi',2.2,'Kphi',0.4,'Kv',0.01,'Kvi',0.002);

% ---- LQI autopilot: longitudinal [w theta q h] + integral of altitude error ----
il = [3 5 8 12];
Aa = zeros(5);  Aa(1:4,1:4) = A(il,il);  Aa(5,4) = -1;     % z' = h_cmd - h
Ba = [B(il,2); 0];
Ql = diag([1/45^2, 1/deg2rad(10)^2, 1/deg2rad(20)^2, 1/300^2, 1/500^2]);
Rl = 1/deg2rad(10)^2;
D.KL = lqr(Aa,Ba,Ql,Rl);                                    % 1 x 5

% ---- LQI autopilot: lateral [v phi p r psi] + integral of heading error ----
it = [2 4 7 9 6];
At = zeros(6);  At(1:5,1:5) = A(it,it);  At(6,5) = -1;     % z' = psi_cmd - psi
Bt = [B(it,[3 4]); 0 0];
Qt = diag([1/6^2, 1/deg2rad(30)^2, 1/deg2rad(30)^2, 1/deg2rad(10)^2, 1/deg2rad(10)^2, 1/deg2rad(15)^2]);
Rt = diag([1/deg2rad(15)^2, 1/deg2rad(5)^2]);
D.KT = lqr(At,Bt,Qt,Rt);                                    % 2 x 6

D.poles_lqi_lon = eig(Aa - Ba*D.KL);
D.poles_lqi_lat = eig(At - Bt*D.KT);
end

function z = dutch_damping(Kr,Al,Bl,Kp,Tw)
% damping of the dutch-roll pair with the yaw damper (washout) and roll damper on
Ae = zeros(5);  Ae(1:4,1:4) = Al;  Ae(5,4) = 1/Tw;  Ae(5,5) = -1/Tw;
Be = [Bl; 0 0];
Kfb = zeros(2,5);  Kfb(1,3) = -Kp;  Kfb(2,4) = Kr;  Kfb(2,5) = -Kr;
e = eig(Ae + Be*Kfb);
if any(real(e) >= 0), z = 0; return; end
c = e(imag(e) > 0.05);
if isempty(c), z = 0; else, z = min(-real(c)./abs(c)); end
end

function [A,B] = numjac_f16(x0,u0,P)
nx = numel(x0); nu = numel(u0);
A = zeros(nx); B = zeros(nx,nu);
for i = 1:nx
    dx = zeros(nx,1); dx(i) = 1e-5*max(1,abs(x0(i)));
    A(:,i) = (f16_dynamics(x0+dx,u0,P) - f16_dynamics(x0-dx,u0,P))/(2*dx(i));
end
for i = 1:nu
    du = zeros(nu,1); du(i) = 1e-5;
    B(:,i) = (f16_dynamics(x0,u0+du,P) - f16_dynamics(x0,u0-du,P))/(2*du(i));
end
end