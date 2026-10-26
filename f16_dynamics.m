function [xdot, out] = f16_dynamics(x, ctrl, P, wind)
% Nonlinear 6-DOF F-16 model, body axes, flat earth.
%
% x    = [u v w phi theta psi p q r pn pe h pow]'   (13 states)
%         u,v,w  body velocities (ft/s)     phi,theta,psi Euler angles (rad)
%         p,q,r  body rates (rad/s)         pn,pe north/east (ft), h altitude (ft)
%         pow    engine power (%)
% ctrl = [throttle(0-1); elevator; aileron; rudder]  (surfaces in rad)
%         elevator: positive = trailing edge down = nose down
%
% wind (optional) = [uw vw ww] wind velocity in body axes (ft/s), default [0 0 0].
%         Used by step 5 for gusts. Existing 3-argument calls are unchanged.
%
% xdot = state derivatives, out = struct of useful outputs (alpha, beta, nz...)

if nargin < 4, wind = [0 0 0]; end

ub = x(1); vb = x(2); wb = x(3);
phi = x(4); th = x(5); psi = x(6);
p = x(7); q = x(8); r = x(9);
h = x(12); pw = x(13);

thr = min(max(ctrl(1),0),1);
de  = min(max(ctrl(2),-P.de_lim),P.de_lim);
da  = min(max(ctrl(3),-P.da_lim),P.da_lim);
dr  = min(max(ctrl(4),-P.dr_lim),P.dr_lim);

% --- air data (ISA troposphere) ---
ua = ub - wind(1);  va = vb - wind(2);  wa = wb - wind(3);   % air-relative velocity
V     = max(sqrt(ua^2+va^2+wa^2), 1);
alpha = atan2(wa, ua);
beta  = asin(va/V);
Tmp   = 518.67 - 3.5662e-3*min(h,36000);          % deg R
rho   = 0.0023769*(Tmp/518.67)^4.2561;            % slug/ft^3
a_snd = sqrt(1.4*1716.5*Tmp);
qbar  = 0.5*rho*V^2;

% --- nondimensional rates ---
pb = p*P.b/(2*V);  qb = q*P.cbar/(2*V);  rb = r*P.b/(2*V);

% --- coefficients ---
CL = P.CL0 + P.CLa*alpha + P.CLde*de;
CD = P.CD0 + P.k*CL^2;
CY = P.CYb*beta + P.CYdr*dr;
Cl = P.Clb*beta + P.Clp*pb + P.Clr*rb + P.Clda*da + P.Cldr*dr;
Cm = P.Cm0 + P.Cma*alpha + P.Cmq*qb + P.Cmde*de;
Cn = P.Cnb*beta + P.Cnp*pb + P.Cnr*rb + P.Cnda*da + P.Cndr*dr;

L = qbar*P.S*CL;  D = qbar*P.S*CD;  Y = qbar*P.S*CY;
T = P.Tmax*(pw/100)*(rho/0.0023769)^0.7;

% --- forces in body axes (wind -> body) ---
ca = cos(alpha); sa = sin(alpha); cb = cos(beta); sb = sin(beta);
Fx = -D*ca*cb - Y*ca*sb + L*sa + T;
Fy = -D*sb + Y*cb;
Fz = -D*sa*cb - Y*sa*sb - L*ca;

% --- translational dynamics ---
udot = r*vb - q*wb - P.g*sin(th)           + Fx/P.m;
vdot = p*wb - r*ub + P.g*cos(th)*sin(phi)  + Fy/P.m;
wdot = q*ub - p*vb + P.g*cos(th)*cos(phi)  + Fz/P.m;

% --- rotational dynamics ---
lm = qbar*P.S*P.b*Cl;  mm = qbar*P.S*P.cbar*Cm;  nm = qbar*P.S*P.b*Cn;
G  = P.Ixx*P.Izz - P.Ixz^2;
G1 = P.Ixz*(P.Ixx - P.Iyy + P.Izz)/G;
G2 = (P.Izz*(P.Izz - P.Iyy) + P.Ixz^2)/G;
G3 = P.Izz/G;   G4 = P.Ixz/G;
G5 = (P.Izz - P.Ixx)/P.Iyy;   G6 = P.Ixz/P.Iyy;
G7 = (P.Ixx*(P.Ixx - P.Iyy) + P.Ixz^2)/G;   G8 = P.Ixx/G;

pdot = G1*p*q - G2*q*r + G3*lm + G4*nm;
qdot = G5*p*r - G6*(p^2 - r^2) + mm/P.Iyy;
rdot = G7*p*q - G1*q*r + G4*lm + G8*nm;

% --- kinematics ---
phidot = p + tan(th)*(q*sin(phi) + r*cos(phi));
thdot  = q*cos(phi) - r*sin(phi);
psidot = (q*sin(phi) + r*cos(phi))/cos(th);

cph = cos(phi); sph = sin(phi); cth = cos(th); sth = sin(th);
cps = cos(psi); sps = sin(psi);
pndot = ub*cth*cps + vb*(sph*sth*cps - cph*sps) + wb*(cph*sth*cps + sph*sps);
pedot = ub*cth*sps + vb*(sph*sth*sps + cph*cps) + wb*(cph*sth*sps - sph*cps);
hdot  = ub*sth - vb*sph*cth - wb*cph*cth;

% --- engine lag ---
powdot = (100*thr - pw)/P.tau_eng;

xdot = [udot; vdot; wdot; phidot; thdot; psidot; pdot; qdot; rdot; ...
        pndot; pedot; hdot; powdot];

if nargout > 1
    out.V = V;  out.alpha = alpha;  out.beta = beta;
    out.mach = V/a_snd;  out.qbar = qbar;
    out.nz = -Fz/(P.m*P.g);          % load factor (g)
end
end