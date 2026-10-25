% STEP 2: Pitch stability augmentation system (SAS) using LQR
% Feeds back w (~ angle of attack) and pitch rate q to the elevator, then
% compares the bare (unstable) airframe against the augmented airframe.
% Needs Control System Toolbox (lqr). Saves f16_step2_sas.png in the current folder.
clear; clc; close all

P  = f16_params();
V0 = 500;  h0 = 0;  tf = 5;
[x0,u0,info] = f16_trim(V0,h0,P);
fprintf('Trim: alpha = %.2f deg, elevator = %.2f deg\n', info.alpha_deg, info.elevator_deg);

% ---- linear model and short-period subsystem [w q] ----
[A,B] = numjac_f16(x0,u0,P);
sp   = [3 8];                         % states w, q
Asp  = A(sp,sp);   Bsp = B(sp,2);     % input = elevator
fprintf('Open-loop short-period poles:\n'); disp(eig(Asp))

% ---- LQR design (Bryson-style weights: 1 / (max acceptable value)^2) ----
Q = diag([1/(V0*deg2rad(5))^2, 1/deg2rad(30)^2]);   % 5 deg alpha, 30 deg/s q
R = 1/deg2rad(10)^2;                                 % 10 deg elevator
K = lqr(Asp,Bsp,Q,R);                                % de = -K*[dw; dq]
fprintf('Gains: K_w = %.4f rad/(ft/s), K_q = %.4f rad/(rad/s)\n', K(1), K(2));
fprintf('Closed-loop short-period poles:\n'); disp(eig(Asp - Bsp*K))

lon = [1 3 5 8];  Kfull = [0 K(1) 0 K(2)];           % u w theta q
fprintf('Closed-loop longitudinal poles (u w theta q):\n');
disp(eig(A(lon,lon) - B(lon,2)*Kfull))

% ---- simulate the same elevator doublet, with and without SAS ----
amp = deg2rad(1.5);
dd  = @(t) -amp*(t>=1 & t<1.5) + amp*(t>=1.5 & t<2);   % negative = nose up

ctrl_ol = @(t,x) [u0(1); u0(2) + dd(t); 0; 0];
ctrl_cl = @(t,x) [u0(1); u0(2) + dd(t) - K*[x(3)-x0(3); x(8)-x0(8)]; 0; 0];

R_ol = simulate(ctrl_ol, x0, tf, P);
R_cl = simulate(ctrl_cl, x0, tf, P);

% ---- plot ----
fig = figure('Color','w','Position',[100 100 950 650],'Visible','on');
try, theme(fig,'light'); catch, end          % R2025a+ dark-mode fix (ignored on older MATLAB)
tl = tiledlayout(fig,2,2,'TileSpacing','compact');
names  = {'\alpha (deg)','q (deg/s)','Elevator (deg)','n_z (g)'};
fields = {'alpha','q','de','nz'};
for i = 1:4
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
    plot(ax, R_ol.t, R_ol.(fields{i}), 'LineWidth',1.8);
    plot(ax, R_cl.t, R_cl.(fields{i}), 'LineWidth',1.8);
    ylabel(ax,names{i});
    if i > 2, xlabel(ax,'Time (s)'); end
    if i == 1, legend(ax,{'SAS off (bare airframe)','SAS on (LQR)'},'Location','best'); end
end
title(tl,'F-16 pitch response to elevator doublet: SAS off vs on');

% ---- force readable colors: white background, black text ----
style_light(fig, tl);

drawnow; figure(fig);
outfile = fullfile(pwd,'f16_step2_sas.png');
exportgraphics(fig, outfile, 'Resolution',200, 'BackgroundColor','white');
fprintf('\nFigure saved to: %s\n', outfile);

% ---------- local functions ----------
function style_light(fig, tl)
try, theme(fig,'light'); catch, end
for a = findall(fig,'Type','axes')'
    set(a,'Color','w','XColor','k','YColor','k','GridColor',[0.4 0.4 0.4],'GridAlpha',0.25);
    a.XLabel.Color = 'k';  a.YLabel.Color = 'k';
end
for l = findall(fig,'Type','legend')'
    set(l,'TextColor','k','Color','w','EdgeColor',[0.7 0.7 0.7]);
end
tl.Title.Color = 'k';
end

function R = simulate(ctrl, x0, tf, P)
opts = odeset('RelTol',1e-8,'AbsTol',1e-9);
[t,X] = ode45(@(t,x) f16_dynamics(x,ctrl(t,x),P), [0 tf], x0, opts);
n = numel(t);
R.t = t; R.alpha = zeros(n,1); R.nz = zeros(n,1); R.de = zeros(n,1);
R.q = rad2deg(X(:,8));
for k = 1:n
    c = ctrl(t(k), X(k,:)');
    [~,o] = f16_dynamics(X(k,:)', c, P);
    R.alpha(k) = rad2deg(o.alpha);
    R.nz(k)    = o.nz;
    R.de(k)    = rad2deg(min(max(c(2),-P.de_lim),P.de_lim));
end
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