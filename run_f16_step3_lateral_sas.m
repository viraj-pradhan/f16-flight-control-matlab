% STEP 3: Lateral-directional stability augmentation (yaw damper + roll damper)
% Yaw damper : yaw rate r -> rudder, through a washout filter (tau = 1 s) so the
%              damper fights dutch-roll oscillation but not steady turns.
% Roll damper: roll rate p -> aileron.
% The pitch SAS from step 2 stays ON in both runs (bare airframe is pitch-unstable),
% so the comparison isolates the lateral dampers.
% Needs Control System Toolbox (lqr). Saves f16_step3_lateral_sas.png in the current folder.

clear; clc; close all
P  = f16_params();
V0 = 500; h0 = 0; tf = 10;
[x0,u0,info] = f16_trim(V0,h0,P);
fprintf('Trim: alpha = %.2f deg, elevator = %.2f deg\n', info.alpha_deg, info.elevator_deg);

% ---- linear model ----
[A,B] = numjac_f16(x0,u0,P);

% pitch SAS from step 2 (same LQR weights, kept on in every run below)
sp  = [3 8];
Asp = A(sp,sp); Bsp = B(sp,2);
Qp  = diag([1/(V0*deg2rad(5))^2, 1/deg2rad(30)^2]);
Rp  = 1/deg2rad(10)^2;
Klon = lqr(Asp,Bsp,Qp,Rp);
fprintf('Pitch SAS gains (from step 2): K_w = %.4f, K_q = %.4f\n', Klon(1), Klon(2));

% lateral-directional subsystem: states [v phi p r], inputs [aileron rudder]
lat = [2 4 7 9];
Al  = A(lat,lat); Bl = B(lat,[3 4]);
fprintf('\nOpen-loop lateral poles:\n'); disp(eig(Al))
[z0,lam0] = dr_mode(Al);
fprintf('Open-loop dutch roll: %.3f %+.3fi  (damping %.3f, wn %.2f rad/s)\n', ...
        real(lam0), imag(lam0), z0, abs(lam0));

% ---- damper design ----
tw = 1.0;      % washout time constant (s)
Kp = 0.10;     % roll damper gain, rad aileron per rad/s roll rate (speeds up roll mode)
% yaw damper gain: pick the K_r that gives the highest dutch-roll damping
Kr = fminbnd(@(k) -dr_mode(lat_cl(Al,Bl,tw,Kp,k)), 0.1, 3);
Acl = lat_cl(Al,Bl,tw,Kp,Kr);
[z1,lam1] = dr_mode(Acl);
fprintf('\nDamper gains: K_p = %.3f rad/(rad/s), K_r = %.3f rad/(rad/s)\n', Kp, Kr);
fprintf('Closed-loop lateral poles (v phi p r washout):\n'); disp(eig(Acl))
fprintf('Closed-loop dutch roll: %.3f %+.3fi  (damping %.3f, wn %.2f rad/s)\n', ...
        real(lam1), imag(lam1), z1, abs(lam1));

% ---- simulate a rudder doublet, dampers off vs on ----
amp = deg2rad(2);
rd  = @(t) amp*(t>=1 & t<1.5) - amp*(t>=1.5 & t<2);   % positive = TE left -> nose left
R_off = simulate(x0,u0,Klon,0, 0, tw,rd,tf,P);
R_on  = simulate(x0,u0,Klon,Kp,Kr,tw,rd,tf,P);
fprintf('\nPeak |beta|: off %.2f deg, on %.2f deg\n', max(abs(R_off.beta)), max(abs(R_on.beta)));
fprintf('Peak |p|   : off %.2f deg/s, on %.2f deg/s\n', max(abs(R_off.p)), max(abs(R_on.p)));
fprintf('beta inside 10%% of peak after: off %.1f s, on %.1f s\n', settle(R_off), settle(R_on));

% ---- plot ----
fig = figure('Color','w','Position',[100 100 1000 780],'Visible','on');
try theme(fig,'light'); catch, end
tl = tiledlayout(fig,3,2,'TileSpacing','compact');
names  = {'\beta sideslip (deg)','p roll rate (deg/s)','r yaw rate (deg/s)', ...
          '\phi bank angle (deg)','Rudder (deg)','Aileron (deg)'};
fields = {'beta','p','r','phi','dr','da'};
for i = 1:6
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
    plot(ax, R_off.t, R_off.(fields{i}), 'LineWidth',1.8);
    plot(ax, R_on.t,  R_on.(fields{i}),  'LineWidth',1.8);
    ylabel(ax,names{i});
    if i > 4, xlabel(ax,'Time (s)'); end
    if i == 1, legend(ax,{'Dampers off','Yaw + roll damper on'},'Location','best'); end
end
title(tl,sprintf(['F-16 lateral response to rudder doublet (pitch SAS on)\n' ...
      'Dutch roll damping %.2f \\rightarrow %.2f'], z0, z1));
style_light(fig, tl);
drawnow; figure(fig);
outfile = fullfile(pwd,'f16_step3_lateral_sas.png');
exportgraphics(fig, outfile, 'Resolution',200, 'BackgroundColor','white');
fprintf('\nFigure saved to: %s\n', outfile);

% ---------- local functions ----------
function M = lat_cl(Al,Bl,tw,Kp,Kr)
% augmented state [v phi p r xw]; washout: xw_dot = (r - xw)/tw, r_wo = r - xw
% da = -Kp*p ; dr = +Kr*r_wo  (Cn_dr < 0, so +rudder makes nose-left moment and opposes +r)
Aa = [Al zeros(4,1); 0 0 0 1/tw -1/tw];
Ba = [Bl; 0 0];
K  = [0 0 -Kp 0 0;
      0 0 0  Kr -Kr];
M  = Aa + Ba*K;
end

function [z,lam] = dr_mode(M)
% dutch roll = least-damped oscillatory pole
e = eig(M); c = e(abs(imag(e)) > 1e-6);
if isempty(c), z = 1; lam = NaN; return; end
zz = -real(c)./abs(c);
[z,i] = min(zz); lam = c(i);
if imag(lam) < 0, lam = conj(lam); end
end

function ts = settle(R)
pk = max(abs(R.beta));
ts = R.t(find(abs(R.beta) > 0.1*pk, 1, 'last'));
end

function R = simulate(x0,u0,Klon,Kp,Kr,tw,rd,tf,P)
ctrl = @(t,xa) [u0(1);
                u0(2) - Klon*[xa(3)-x0(3); xa(8)-x0(8)];
                -Kp*xa(7);
                rd(t) + Kr*(xa(9) - xa(14))];
f = @(t,xa) [f16_dynamics(xa(1:13), ctrl(t,xa), P); (xa(9) - xa(14))/tw];
opts = odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.02);
[t,X] = ode45(f, [0 tf], [x0; 0], opts);
n = numel(t);
R.t = t; R.beta = zeros(n,1); R.da = zeros(n,1); R.dr = zeros(n,1);
R.p = rad2deg(X(:,7)); R.r = rad2deg(X(:,9)); R.phi = rad2deg(X(:,4));
for k = 1:n
    c = ctrl(t(k), X(k,:)');
    [~,o] = f16_dynamics(X(k,1:13)', c, P);
    R.beta(k) = rad2deg(o.beta);
    R.da(k) = rad2deg(min(max(c(3),-P.da_lim),P.da_lim));
    R.dr(k) = rad2deg(min(max(c(4),-P.dr_lim),P.dr_lim));
end
end

function style_light(fig, tl)
try theme(fig,'light'); catch, end
for a = findall(fig,'Type','axes')'
    set(a,'Color','w','XColor','k','YColor','k','GridColor',[0.4 0.4 0.4],'GridAlpha',0.25);
    a.XLabel.Color = 'k'; a.YLabel.Color = 'k';
end
for l = findall(fig,'Type','legend')'
    set(l,'TextColor','k','Color','w','EdgeColor',[0.7 0.7 0.7]);
end
tl.Title.Color = 'k';
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