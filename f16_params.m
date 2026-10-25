function P = f16_params()
% F-16 parameters (US units: ft, slug, lb, rad, s)
% Mass properties and geometry follow the Stevens & Lewis F-16.
% Aerodynamics here are SIMPLIFIED linear coefficients so the model runs
% immediately. Later, replace the coefficient block with the NASA TP-1538
% lookup tables (functions of alpha, beta, elevator).

% --- mass, inertia, geometry ---
P.m    = 20500/32.174;   % slug
P.Ixx  = 9496;           % slug-ft^2
P.Iyy  = 55814;
P.Izz  = 63100;
P.Ixz  = 982;
P.S    = 300;            % wing area, ft^2
P.b    = 30;             % span, ft
P.cbar = 11.32;          % mean chord, ft
P.g    = 32.174;         % ft/s^2

% --- engine ---
P.Tmax    = 25000;       % sea-level max thrust, lb
P.tau_eng = 1.0;         % engine lag, s

% --- longitudinal coefficients (per rad) ---
P.CL0  = 0.05;   P.CLa  = 4.6;    P.CLde = 0.30;
P.CD0  = 0.022;  P.k    = 0.124;               % CD = CD0 + k*CL^2
P.Cm0  = 0.0;
P.Cma  = 0.10;   % >0 = statically UNSTABLE (relaxed stability, like the real F-16)
P.Cmq  = -3.0;   P.Cmde = -0.57;

% --- lateral-directional coefficients (per rad) ---
P.CYb  = -1.15;  P.CYdr = 0.20;
P.Clb  = -0.10;  P.Clp  = -0.40;  P.Clr  = 0.10;  P.Clda = 0.10;  P.Cldr = 0.02;
P.Cnb  = 0.12;   P.Cnp  = -0.03;  P.Cnr  = -0.30; P.Cnda = 0.0;   P.Cndr = -0.08;

% --- control limits (rad) ---
P.de_lim = deg2rad(25);
P.da_lim = deg2rad(21.5);
P.dr_lim = deg2rad(30);
end