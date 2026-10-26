function [xdot, out] = f16_dynamics_nasa(x, ctrl, P)
% Nonlinear 6-DOF F-16 model using the NASA TP-1538 aerodynamic tables
% (f16_aero_nasa.m). Same interface, state vector, units and sign conventions
% as f16_dynamics.m, so it can be swapped in directly:
%
% x    = [u v w phi theta psi p q r pn pe h pow]'  (13 states)
% ctrl = [throttle(0-1); elevator; aileron; rudder] (surfaces in rad)
%        elevator +  = trailing edge down = nose down
%        aileron  +  = right roll (project convention; flipped internally
%                      because Stevens & Lewis use the opposite sign)
%        rudder   +  = nose left
% P    = f16_params(); optional field P.xcg = centre of gravity in fractions
%        of mean chord (default 0.35, the Stevens & Lewis reference value).
%        Moving it aft (larger) makes the aircraft less stable.
%
% Only the aerodynamics differ from f16_dynamics.m. Mass, inertia, geometry,
% atmosphere and the engine model are the same, so comparisons isolate the
% effect of the real aerodynamic data.

if isfield(P,'xcg'), xcg = P.xcg; else, xcg = 0.35; end
xcgr = 0.35;                        % reference CG of the tables

ub = x(1); vb = x(2); wb = x(3);
phi = x(4); th = x(5); psi = x(6);
p = x(7); q = x(8); r = x(9);
h = x(12); pw = x(13);

thr = min(max(ctrl(1),0),1);
de  = min(max(ctrl(2),-P.de_lim),P.de_lim);
da  = min(max(ctrl(3),-P.da_lim),P.da_lim);
dr  = min(max(ctrl(4),-P.dr_lim),P.dr_lim);

% --- air data (ISA troposphere) ---
V     = max(sqrt(ub^2+vb^2+wb^2), 1);
alpha = atan2(wb, ub);
beta  = asin(vb/V);
Tmp   = 518.67 - 3.5662e-3*min(h,36000);
rho   = 0.0023769*(Tmp/518.67)^4.2561;
a_snd = sqrt(1.4*1716.5*Tmp);
qbar  = 0.5*rho*V^2;

% --- table look-up (degrees, Stevens & Lewis signs) ---
[C, D] = f16_aero_nasa(rad2deg(alpha), rad2deg(beta), rad2deg(de), -rad2deg(da), rad2deg(dr));

% --- damping terms and CG shift ---
t2v = 0.5/V;  cq = P.cbar*q*t2v;  b2v = P.b*t2v;
Cx = C.Cx + cq*D(1);
Cy = C.Cy + b2v*(D(2)*r + D(3)*p);
Cz = C.Cz + cq*D(4);
Cl = C.Cl + b2v*(D(5)*r + D(6)*p);
Cm = C.Cm + cq*D(7) + Cz*(xcgr - xcg);
Cn = C.Cn + b2v*(D(8)*r + D(9)*p) - Cy*(xcgr - xcg)*P.cbar/P.b;

% --- forces (tables are already in body axes) ---
T  = P.Tmax*(pw/100)*(rho/0.0023769)^0.7;
Fx = qbar*P.S*Cx + T;
Fy = qbar*P.S*Cy;
Fz = qbar*P.S*Cz;

% --- translational dynamics ---
udot = r*vb - q*wb - P.g*sin(th) + Fx/P.m;
vdot = p*wb - r*ub + P.g*cos(th)*sin(phi) + Fy/P.m;
wdot = q*ub - p*vb + P.g*cos(th)*cos(phi) + Fz/P.m;

% --- rotational dynamics ---
lm = qbar*P.S*P.b*Cl;  mm = qbar*P.S*P.cbar*Cm;  nm = qbar*P.S*P.b*Cn;
G  = P.Ixx*P.Izz - P.Ixz^2;
G1 = P.Ixz*(P.Ixx - P.Iyy + P.Izz)/G;
G2 = (P.Izz*(P.Izz - P.Iyy) + P.Ixz^2)/G;
G3 = P.Izz/G;  G4 = P.Ixz/G;
G5 = (P.Izz - P.Ixx)/P.Iyy;  G6 = P.Ixz/P.Iyy;
G7 = (P.Ixx*(P.Ixx - P.Iyy) + P.Ixz^2)/G;  G8 = P.Ixx/G;
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
    out.V = V; out.alpha = alpha; out.beta = beta;
    out.mach = V/a_snd; out.qbar = qbar;
    out.nz = -Fz/(P.m*P.g);
end
end