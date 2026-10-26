% STEP 1: trim -> open-loop modes -> elevator doublet simulation
% Compares the F-16-like unstable airframe against a statically stable one.
% Saves f16_step1_openloop.png (white background, black text) in the current folder.
clear; clc; close all

V0 = 500;  h0 = 0;                 % trim condition: 500 ft/s, sea level
tf = 5;                            % sim length (s). Linear aero has no stall model,
                                   % so keep this short for the unstable case.
cases = struct('name',{'Relaxed stability (F-16 like)','Statically stable'}, ...
               'Cma', {0.10, -0.20});

fig = figure('Color','w','Position',[100 100 1000 700],'Visible','on');
try theme(fig,'light'); catch, end          % R2025a+ dark-mode fix (ignored on older MATLAB)
tl  = tiledlayout(fig,3,2,'TileSpacing','compact');
ax  = arrayfun(@(i) nexttile(tl), 1:6);
for i = 1:6, hold(ax(i),'on'); grid(ax(i),'on'); end

for c = 1:numel(cases)
    P = f16_params();
    P.Cma = cases(c).Cma;

    % ---- trim ----
    [x0,u0,info] = f16_trim(V0,h0,P);
    fprintf('\n=== %s ===\n', cases(c).name);
    fprintf('Trim: alpha = %.2f deg, throttle = %.3f, elevator = %.2f deg, residual = %.1e\n', ...
            info.alpha_deg, info.throttle, info.elevator_deg, info.residual);

    % ---- linearize (finite differences) and show modes ----
    [A,B] = numjac_f16(x0,u0,P);
    lon = [1 3 5 8];                % u w theta q
    lat = [2 4 7 9];                % v phi p r
    fprintf('Longitudinal poles:\n'); disp(eig(A(lon,lon)))
    fprintf('Lateral poles:\n');      disp(eig(A(lat,lat)))
    damp(ss(A(lon,lon), zeros(4,1), eye(4), 0));

    % ---- elevator doublet (negative = nose up) ----
    amp = deg2rad(1.5);
    ctrl = @(t) u0 + [0; -amp*(t>=1 & t<1.5) + amp*(t>=1.5 & t<2); 0; 0];
    opts = odeset('RelTol',1e-8,'AbsTol',1e-9);
    [t,X] = ode45(@(t,x) f16_dynamics(x,ctrl(t),P), [0 tf], x0, opts);

    % ---- outputs ----
    n = numel(t); al = zeros(n,1); nz = zeros(n,1); Vt = zeros(n,1); de = zeros(n,1);
    for k = 1:n
        ck = ctrl(t(k));
        [~,o] = f16_dynamics(X(k,:)',ck,P);
        al(k) = rad2deg(o.alpha); nz(k) = o.nz; Vt(k) = o.V; de(k) = rad2deg(ck(2));
    end
    plot(ax(1),t,al,'LineWidth',1.8);
    plot(ax(2),t,rad2deg(X(:,8)),'LineWidth',1.8);
    plot(ax(3),t,X(:,12),'LineWidth',1.8);
    plot(ax(4),t,Vt,'LineWidth',1.8);
    plot(ax(5),t,nz,'LineWidth',1.8);
    plot(ax(6),t,de,'LineWidth',1.8);
end

ylabel(ax(1),'\alpha (deg)');        ylabel(ax(2),'q (deg/s)');
ylabel(ax(3),'Altitude (ft)');       ylabel(ax(4),'Airspeed (ft/s)');
ylabel(ax(5),'n_z (g)');             ylabel(ax(6),'Elevator (deg)');
for i = 5:6, xlabel(ax(i),'Time (s)'); end
lg = legend(ax(1),{cases.name},'Location','best');
title(tl,'F-16 open-loop response to elevator doublet (500 ft/s, sea level)');

% ---- force readable colors: white background, black text ----
style_light(fig, tl);

drawnow; figure(fig);
outfile = fullfile(pwd,'f16_step1_openloop.png');
exportgraphics(fig, outfile, 'Resolution',200, 'BackgroundColor','white');
fprintf('\nFigure saved to: %s\n', outfile);

% ---------- local functions ----------
function style_light(fig, tl)
try theme(fig,'light'); catch, end
for a = findall(fig,'Type','axes')'
    set(a,'Color','w','XColor','k','YColor','k','GridColor',[0.4 0.4 0.4],'GridAlpha',0.25);
    a.XLabel.Color = 'k';  a.YLabel.Color = 'k';
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