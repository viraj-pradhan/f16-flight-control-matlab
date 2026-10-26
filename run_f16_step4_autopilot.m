% STEP 4: Autopilot - altitude hold, heading hold, airspeed hold
% Built as nested loops on top of the step 2 pitch SAS and step 3 dampers:
%   Longitudinal: altitude -> climb-rate command -> pitch attitude command -> elevator
%   Lateral     : heading  -> bank angle command  -> aileron   (yaw damper keeps turns coordinated)
%   Speed       : airspeed -> throttle (PI autothrottle)
% Scenario: level at 500 ft/s, sea level. At t = 2 s command +500 ft and +30 deg heading.
% Needs Control System Toolbox (lqr). Saves f16_step4_autopilot.png in the current folder.

clear; clc; close all
P  = f16_params();
V0 = 500; h0 = 0; tf = 60;
[x0,u0,info] = f16_trim(V0,h0,P);
fprintf('Trim: alpha = %.2f deg, throttle = %.3f, elevator = %.2f deg\n', ...
        info.alpha_deg, info.throttle, info.elevator_deg);

% ---- inner loops (same designs as steps 2 and 3) ----
[A,B] = numjac_f16(x0,u0,P);
sp = [3 8];
G.Klon = lqr(A(sp,sp), B(sp,2), diag([1/(V0*deg2rad(5))^2, 1/deg2rad(30)^2]), 1/deg2rad(10)^2);
G.Kp = 0.10;  G.Kr = 0.851;  G.tw = 1.0;     % roll damper, yaw damper, washout

% ---- autopilot gains ----
G.Kth  = 1.5;    G.Kthi = 1.0;    % pitch attitude hold: rad elevator per rad (P, I)
G.Kh   = 0.15;                    % altitude -> climb-rate command (ft/s per ft)
G.Khd  = 0.003;  G.Khdi = 0.001;  % climb rate -> pitch command (rad per ft/s) (P, I)
G.Kphi = 0.4;                     % bank hold: rad aileron per rad bank error
G.Kpsi = 2.5;                     % heading -> bank command (rad per rad)
G.KV   = 0.01;   G.KVi  = 0.002;  % autothrottle (throttle per ft/s) (P, I)

% ---- limits (keeps the manoeuvre gentle and realistic) ----
G.hd_lim  = 50;              % max climb/descent rate command, ft/s
G.th_lim  = deg2rad(10);     % max pitch change from trim
G.phi_lim = deg2rad(30);     % max bank angle

% ---- commands ----
cmd = @(t) struct('h',   h0 + 500*(t>=2), ...
                  'psi', deg2rad(30)*(t>=2), ...
                  'V',   V0);

% ---- simulate ----
% augmented state: [x(13); washout; climb-rate integral; speed integral; pitch integral]
xa0  = [x0; 0; 0; 0; 0];
opts = odeset('RelTol',1e-7,'AbsTol',1e-8,'MaxStep',0.05);
[t,X] = ode45(@(t,xa) aug_dyn(t,xa,x0,u0,G,cmd,P), [0 tf], xa0, opts);

% ---- post-process ----
n = numel(t);
h = X(:,12); psi = rad2deg(X(:,6)); phi = rad2deg(X(:,4)); th = rad2deg(X(:,5));
V = sqrt(sum(X(:,1:3).^2,2));
U = zeros(n,4); beta = zeros(n,1); hc = zeros(n,1); psic = zeros(n,1);
for k = 1:n
    U(k,:) = autopilot(t(k), X(k,:)', x0, u0, G, cmd)';
    U(k,1)   = min(max(U(k,1),0),1);
    U(k,2:4) = [min(max(U(k,2),-P.de_lim),P.de_lim), ...
                min(max(U(k,3),-P.da_lim),P.da_lim), ...
                min(max(U(k,4),-P.dr_lim),P.dr_lim)];
    [~,o] = f16_dynamics(X(k,1:13)', U(k,:)', P);
    beta(k) = rad2deg(o.beta);
    c = cmd(t(k)); hc(k) = c.h; psic(k) = rad2deg(c.psi);
end

fprintf('\nAltitude : peak %.0f ft (overshoot %.1f ft), final %.1f ft, within 25 ft after %.1f s\n', ...
        max(h), max(h)-500, h(end), settle(t,h,500,25));
fprintf('Heading  : peak %.2f deg, final %.2f deg, within 1 deg after %.1f s\n', ...
        max(psi), psi(end), settle(t,psi,30,1));
fprintf('Airspeed : %.1f to %.1f ft/s, final %.1f ft/s\n', min(V), max(V), V(end));
fprintf('Peak bank %.1f deg, peak pitch %.1f deg, peak |sideslip| %.2f deg\n', ...
        max(abs(phi)), max(th), max(abs(beta)));
fprintf('Elevator %.1f to %.1f deg, peak |aileron| %.1f deg, peak |rudder| %.2f deg\n', ...
        min(rad2deg(U(:,2))), max(rad2deg(U(:,2))), max(abs(rad2deg(U(:,3)))), max(abs(rad2deg(U(:,4)))));
fprintf('Throttle %.3f to %.3f\n', min(U(:,1)), max(U(:,1)));

% ---- plot ----
fig = figure('Color','w','Position',[100 100 1050 800],'Visible','on');
try theme(fig,'light'); catch, end
tl = tiledlayout(fig,3,2,'TileSpacing','compact');

ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
plot(ax,t,hc,'k--','LineWidth',1.2); plot(ax,t,h,'LineWidth',1.8);
ylabel(ax,'Altitude (ft)'); legend(ax,{'Command','Response'},'Location','southeast');
pad_ylim(ax,[h;hc]);

ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
plot(ax,t,psic,'k--','LineWidth',1.2); plot(ax,t,psi,'LineWidth',1.8);
ylabel(ax,'Heading \psi (deg)'); legend(ax,{'Command','Response'},'Location','southeast');
pad_ylim(ax,[psi;psic]);

ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
plot(ax,t,V0*ones(n,1),'k--','LineWidth',1.2); plot(ax,t,V,'LineWidth',1.8);
ylabel(ax,'Airspeed (ft/s)'); pad_ylim(ax,[V;V0]);

ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
plot(ax,t,phi,'LineWidth',1.8); plot(ax,t,th,'LineWidth',1.8); plot(ax,t,beta,'LineWidth',1.8);
ylabel(ax,'Angle (deg)'); legend(ax,{'Bank \phi','Pitch \theta','Sideslip \beta'},'Location','northeast');
pad_ylim(ax,[phi;th;beta]);

ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
S = rad2deg(U(:,2:4));
plot(ax,t,S(:,1),'LineWidth',1.8); plot(ax,t,S(:,2),'LineWidth',1.8); plot(ax,t,S(:,3),'LineWidth',1.8);
ylabel(ax,'Surface (deg)'); xlabel(ax,'Time (s)');
legend(ax,{'Elevator','Aileron','Rudder'},'Location','southeast'); pad_ylim(ax,S(:));

ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
plot(ax,t,U(:,1),'LineWidth',1.8);
ylabel(ax,'Throttle (0-1)'); xlabel(ax,'Time (s)'); pad_ylim(ax,U(:,1));

title(tl,'F-16 autopilot: +500 ft climb and 30\circ heading change (altitude, heading, airspeed hold)');
style_light(fig, tl);
drawnow; figure(fig);
outfile = fullfile(pwd,'f16_step4_autopilot.png');
exportgraphics(fig, outfile, 'Resolution',200, 'BackgroundColor','white');
fprintf('\nFigure saved to: %s\n', outfile);

% ---------- local functions ----------
function [u, dint] = autopilot(t, xa, x0, u0, G, cmd)
% returns control vector u and the derivatives of the 4 controller states
x = xa(1:13); xw = xa(14); ihd = xa(15); iV = xa(16); ith = xa(17);
c = cmd(t);
ub = x(1); vb = x(2); wb = x(3); phi = x(4); th = x(5); psi = x(6);
p = x(7); q = x(8); r = x(9); h = x(12);
V    = sqrt(ub^2 + vb^2 + wb^2);
hdot = ub*sin(th) - vb*sin(phi)*cos(th) - wb*cos(phi)*cos(th);

% longitudinal: altitude -> climb rate -> pitch attitude -> elevator (+ pitch SAS)
hd_c  = sat(G.Kh*(c.h - h), G.hd_lim);
th_c  = x0(5) + sat(G.Khd*(hd_c - hdot) + G.Khdi*ihd, G.th_lim);
de    = u0(2) - G.Klon*[wb - x0(3); q] + G.Kth*(th - th_c) + G.Kthi*ith;   % +de = nose down

% lateral: heading -> bank -> aileron (+ roll damper); yaw damper on rudder
epsi  = atan2(sin(c.psi - psi), cos(c.psi - psi));   % wrapped heading error
phi_c = sat(G.Kpsi*epsi, G.phi_lim);
da    = -G.Kp*p - G.Kphi*(phi - phi_c);
r_wo  = r - xw;
dr    = G.Kr*r_wo;

% speed: autothrottle
thr   = u0(1) + G.KV*(c.V - V) + G.KVi*iV;

u    = [thr; de; da; dr];
dint = [r_wo/G.tw; hd_c - hdot; c.V - V; th - th_c];
end

function dxa = aug_dyn(t, xa, x0, u0, G, cmd, P)
[u, dint] = autopilot(t, xa, x0, u0, G, cmd);
dxa = [f16_dynamics(xa(1:13), u, P); dint];
end

function y = sat(x, lim)
y = min(max(x, -lim), lim);
end

function ts = settle(t, y, target, tol)
i = find(abs(y - target) > tol, 1, 'last');
if isempty(i), ts = 0; else, ts = t(i); end
end

function pad_ylim(ax, y)
lo = min(y); hi = max(y); d = hi - lo;
if d < 1e-6, d = max(abs(hi),1)*0.1; end
ylim(ax, [lo - 0.08*d, hi + 0.08*d]);
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