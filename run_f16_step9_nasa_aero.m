% STEP 9: NASA TP-1538 aerodynamic tables
% Replaces the simplified linear aerodynamics with the F-16 wind-tunnel data
% (Stevens & Lewis tabulation) and checks what changes:
%   1. Trim and open-loop modes: simplified model vs NASA tables
%   2. Validation: trim at 502 ft/s against the published Stevens & Lewis result
%   3. Lift, drag and pitching moment curves (stall and relaxed stability)
%   4. Stability vs centre-of-gravity position, with and without the step 2 pitch SAS
%   5. Elevator doublet at an aft CG: bare airframe, step 2 SAS gains, re-designed SAS
% Earlier steps are untouched: they keep using f16_dynamics.m.
% Needs Control System Toolbox (lqr). Saves f16_step9_aero.png and f16_step9_nasa_sas.png.

clear; clc; close all
P  = f16_params();
V0 = 500; h0 = 0;
f_simple = @(x,u) f16_dynamics(x,u,P);
f_nasa   = @(x,u) f16_dynamics_nasa(x,u,P);          % P has no xcg -> 0.35
lon = [1 3 5 8]; lat = [2 4 7 9]; sp = [3 8];

% ---- 1. trim and modes ----
fprintf('=== Trim at %d ft/s, sea level (CG 0.35 for NASA model) ===\n', V0);
[xs,us,rs] = trim_model(f_simple, V0, h0);
[xn,un,rn] = trim_model(f_nasa,   V0, h0);
fprintf('%-18s alpha %5.2f deg  throttle %.3f  elevator %6.2f deg  residual %.1e\n', ...
        'Simplified model', rad2deg(xs(5)), us(1), rad2deg(us(2)), rs);
fprintf('%-18s alpha %5.2f deg  throttle %.3f  elevator %6.2f deg  residual %.1e\n', ...
        'NASA tables', rad2deg(xn(5)), un(1), rad2deg(un(2)), rn);
[As,~]  = numjac(f_simple, xs, us);
[An,Bn] = numjac(f_nasa,   xn, un);
fprintf('\nLongitudinal poles [u w theta q]:\n');
fprintf('  simplified : %s\n', poles_str(eig(As(lon,lon))));
fprintf('  NASA tables: %s\n', poles_str(eig(An(lon,lon))));
fprintf('Lateral poles [v phi p r]:\n');
fprintf('  simplified : %s\n', poles_str(eig(As(lat,lat))));
fprintf('  NASA tables: %s\n', poles_str(eig(An(lat,lat))));
lam = max(real(eig(An(lon,lon))));
if lam > 0
    fprintf('NASA model is unstable at CG 0.35: pole +%.3f /s, doubling time %.1f s\n', lam, log(2)/lam);
end

% ---- 2. validation against Stevens & Lewis ----
[xv,uv] = trim_model(f_nasa, 502, 0);
fprintf('\n=== Validation: 502 ft/s, sea level, CG 0.35 ===\n');
fprintf('This model      : alpha %.3f deg, elevator %.3f deg\n', rad2deg(xv(5)), rad2deg(uv(2)));
fprintf('Stevens & Lewis : alpha 2.12 deg,  elevator -0.76 deg (published trim)\n');
fprintf('(Throttle differs because this project keeps its own simpler engine model.)\n');

% ---- 3. aerodynamic curves ----
al = -10:0.5:45;  n = numel(al);
CLn = zeros(n,1); CDn = CLn; CLs = CLn; CDs = CLn;
Cm_cg = zeros(n,3); cgs = [0.30 0.35 0.38]; Cms = zeros(n,1);
for k = 1:n
    C = f16_aero_nasa(al(k), 0, 0, 0, 0);
    a = deg2rad(al(k));
    CLn(k) = C.Cx*sin(a) - C.Cz*cos(a);
    CDn(k) = -C.Cx*cos(a) - C.Cz*sin(a);
    for j = 1:3, Cm_cg(k,j) = C.Cm + C.Cz*(0.35 - cgs(j)); end
    CLs(k) = P.CL0 + P.CLa*a;  CDs(k) = P.CD0 + P.k*CLs(k)^2;  Cms(k) = P.Cm0 + P.Cma*a;
end
[CLmax,i] = max(CLn);
fprintf('\nNASA tables: maximum lift coefficient %.2f at alpha %.1f deg (stall).\n', CLmax, al(i));
fprintf('Simplified model has no stall: CL keeps rising linearly.\n');

% ---- 4. stability vs CG ----
Kfix = lqr(As(sp,sp), numjac_col(f_simple,xs,us), ...
           diag([1/(V0*deg2rad(5))^2, 1/deg2rad(30)^2]), 1/deg2rad(10)^2);   % step 2 gains
fprintf('\nStep 2 pitch SAS gains (designed on simplified model): K_w = %.4f, K_q = %.4f\n', Kfix(1), Kfix(2));
cg_list = 0.25:0.025:0.40;  m = numel(cg_list);
ol_max = zeros(m,1); cl_max = zeros(m,1); cl_zeta = zeros(m,1);
fprintf('\n  CG     open-loop max Re   with step 2 SAS: max Re   short-period damping\n');
for j = 1:m
    Pj = P; Pj.xcg = cg_list(j);
    fj = @(x,u) f16_dynamics_nasa(x,u,Pj);
    [xj,uj] = trim_model(fj, V0, h0);
    [Aj,Bj] = numjac(fj, xj, uj);
    ol_max(j) = max(real(eig(Aj(lon,lon))));
    Acl = Aj - Bj(:,2)*[0 0 Kfix(1) 0 0 0 0 Kfix(2) 0 0 0 0 0];
    cl_max(j) = max(real(eig(Acl(lon,lon))));
    e = eig(Acl(sp,sp));  cl_zeta(j) = min(-real(e)./abs(e));
    fprintf('  %.3f   %+8.3f            %+8.4f                 %.2f\n', cg_list(j), ol_max(j), cl_max(j), cl_zeta(j));
end

% ---- 5. doublet at aft CG ----
cg_demo = 0.38;
Pd = P; Pd.xcg = cg_demo;
fd = @(x,u) f16_dynamics_nasa(x,u,Pd);
[xd,ud] = trim_model(fd, V0, h0);
[Ad,Bd] = numjac(fd, xd, ud);
ed = eig(Ad(sp,sp));
Knew = lqr(Ad(sp,sp), Bd(sp,2), diag([1/(V0*deg2rad(5))^2, 1/deg2rad(30)^2]), 1/deg2rad(10)^2);
fprintf('\n=== Aft CG %.2f ===\n', cg_demo);
fprintf('Trim: alpha %.2f deg, elevator %.2f deg\n', rad2deg(xd(5)), rad2deg(ud(2)));
fprintf('Open-loop short-period poles: %s  (doubling time %.2f s)\n', poles_str(ed), log(2)/max(real(ed)));
fprintf('Step 2 SAS gains          -> %s\n', poles_str(eig(Ad(sp,sp) - Bd(sp,2)*Kfix)));
fprintf('Re-designed on NASA model -> K_w = %.4f, K_q = %.4f, poles %s\n', Knew(1), Knew(2), ...
        poles_str(eig(Ad(sp,sp) - Bd(sp,2)*Knew)));

amp = deg2rad(1.5);
dd  = @(t) -amp*(t>=1 & t<1.5) + amp*(t>=1.5 & t<2);      % negative = nose up
tf  = 5;
Rb = sim_doublet(fd, xd, ud, [0 0], dd, tf, P);
Rf = sim_doublet(fd, xd, ud, Kfix,  dd, tf, P);
Rn = sim_doublet(fd, xd, ud, Knew,  dd, tf, P);
fprintf('Peak alpha change: bare %.2f deg (still growing), step 2 SAS %.2f deg, re-designed %.2f deg\n', ...
        max(abs(Rb.alpha)), max(abs(Rf.alpha)), max(abs(Rn.alpha)));

% ---- figure 1: aerodynamics and stability ----
fig1 = figure('Color','w','Position',[80 80 1050 760],'Visible','on');
try theme(fig1,'light'); catch, end
tl1 = tiledlayout(fig1,2,2,'TileSpacing','compact');
ax = nexttile(tl1); hold(ax,'on'); grid(ax,'on');
plot(ax,al,CLs,'LineWidth',1.8); plot(ax,al,CLn,'LineWidth',1.8);
plot(ax,al(i),CLmax,'ko','MarkerFaceColor','k');
text(ax,al(i)-4,CLmax-0.35,sprintf('stall: C_{L,max} %.2f',CLmax),'Color','k');
xlabel(ax,'\alpha (deg)'); ylabel(ax,'Lift coefficient C_L');
legend(ax,{'Simplified (linear)','NASA tables'},'Location','southeast'); pad_ylim(ax,[CLs;CLn]);

ax = nexttile(tl1); hold(ax,'on'); grid(ax,'on');
plot(ax,al,CDs,'LineWidth',1.8); plot(ax,al,CDn,'LineWidth',1.8);
xlabel(ax,'\alpha (deg)'); ylabel(ax,'Drag coefficient C_D');
legend(ax,{'Simplified','NASA tables'},'Location','northwest'); pad_ylim(ax,[CDs;CDn]);

ax = nexttile(tl1); hold(ax,'on'); grid(ax,'on');
plot(ax,al,Cms,'k--','LineWidth',1.4);
for j = 1:3, plot(ax,al,Cm_cg(:,j),'LineWidth',1.8); end
yline(ax,0,'Color',[0.5 0.5 0.5]);
xlabel(ax,'\alpha (deg)'); ylabel(ax,'Pitching moment C_m (\delta_e = 0)');
legend(ax,{'Simplified (C_{m\alpha} = +0.10)','NASA, CG 0.30','NASA, CG 0.35','NASA, CG 0.38'},'Location','northwest');
xlim(ax,[-10 30]); pad_ylim(ax,[Cms(al<=30); reshape(Cm_cg(al<=30,:),[],1)]);

ax = nexttile(tl1); hold(ax,'on'); grid(ax,'on');
plot(ax,cg_list,ol_max,'-o','LineWidth',1.8); plot(ax,cg_list,cl_max,'-s','LineWidth',1.8);
yline(ax,0,'Color',[0.5 0.5 0.5]);
xlabel(ax,'CG position (fraction of mean chord)'); ylabel(ax,'Largest real part of poles (1/s)');
legend(ax,{'Bare airframe','With step 2 pitch SAS'},'Location','northwest');
text(ax,0.255,max(ol_max)*0.55,'above 0 = unstable','Color','k');
pad_ylim(ax,[ol_max;cl_max]);
title(tl1,'F-16 with NASA TP-1538 aerodynamic data: stall, relaxed stability and CG effect');
style_light(fig1,tl1);
drawnow;
exportgraphics(fig1, fullfile(pwd,'f16_step9_aero.png'), 'Resolution',200, 'BackgroundColor','white');

% ---- figure 2: doublet at aft CG ----
fig2 = figure('Color','w','Position',[120 120 950 650],'Visible','on');
try theme(fig2,'light'); catch, end
tl2 = tiledlayout(fig2,2,2,'TileSpacing','compact');
names = {'\Delta\alpha (deg)','q (deg/s)','Elevator (deg)','n_z (g)'};
fields = {'alpha','q','de','nz'};
for k = 1:4
    ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
    plot(ax,Rb.t,Rb.(fields{k}),'LineWidth',1.8);
    plot(ax,Rf.t,Rf.(fields{k}),'LineWidth',1.8);
    plot(ax,Rn.t,Rn.(fields{k}),'--','LineWidth',1.8);
    ylabel(ax,names{k}); pad_ylim(ax,[Rb.(fields{k});Rf.(fields{k});Rn.(fields{k})]);
    if k > 2, xlabel(ax,'Time (s)'); end
    if k == 1, legend(ax,{'Bare airframe','Step 2 SAS gains (unchanged)','SAS re-designed on NASA model'},'Location','northwest'); end
end
title(tl2,sprintf('NASA-table F-16, aft CG %.2f: elevator doublet',cg_demo));
style_light(fig2,tl2);
drawnow;
exportgraphics(fig2, fullfile(pwd,'f16_step9_nasa_sas.png'), 'Resolution',200, 'BackgroundColor','white');
fprintf('\nSaved f16_step9_aero.png and f16_step9_nasa_sas.png in %s\n', pwd);

% ---------- local functions ----------
function [x0,u0,res] = trim_model(f, V, h)
% straight and level trim for any dynamics function f(x,u); unknowns [alpha throttle elevator]
opts = optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',20000,'MaxIter',20000);
cost = @(z) trim_cost(f, z, V, h);
z = fminsearch(cost, [0.04; 0.12; 0.01], opts);
z = fminsearch(cost, z, opts);
[x0,u0] = build_state(z, V, h);
xd = f(x0,u0); res = norm(xd([1 3 8]));
end

function J = trim_cost(f, z, V, h)
[x,u] = build_state(z, V, h);
xd = f(x,u);
J = xd(1)^2 + xd(3)^2 + 100*xd(8)^2;
end

function [x,u] = build_state(z, V, h)
a = z(1);
x = [V*cos(a); 0; V*sin(a); 0; a; 0; 0; 0; 0; 0; 0; h; 100*z(2)];
u = [z(2); z(3); 0; 0];
end

function [A,B] = numjac(f, x0, u0)
nx = numel(x0); nu = numel(u0);
A = zeros(nx); B = zeros(nx,nu);
for i = 1:nx
    dx = zeros(nx,1); dx(i) = 1e-5*max(1,abs(x0(i)));
    A(:,i) = (f(x0+dx,u0) - f(x0-dx,u0))/(2*dx(i));
end
for i = 1:nu
    du = zeros(nu,1); du(i) = 1e-5;
    B(:,i) = (f(x0,u0+du) - f(x0,u0-du))/(2*du(i));
end
end

function b = numjac_col(f, x0, u0)
% elevator column of B for the short-period states [w q]
[~,B] = numjac(f, x0, u0); b = B([3 8],2);
end

function R = sim_doublet(f, x0, u0, K, dd, tf, P)
ctrl = @(t,x) [u0(1); u0(2) + dd(t) - K*[x(3)-x0(3); x(8)-x0(8)]; 0; 0];
opts = odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.01);
[t,X] = ode45(@(t,x) f(x, ctrl(t,x)), [0 tf], x0, opts);
n = numel(t);
R.t = t; R.alpha = zeros(n,1); R.nz = zeros(n,1); R.de = zeros(n,1);
R.q = rad2deg(X(:,8));
for k = 1:n
    c = ctrl(t(k), X(k,:)');
    [~,o] = f(X(k,:)', c);
    R.alpha(k) = rad2deg(o.alpha - x0(5));     % change from trim (theta = alpha at trim)
    R.nz(k) = o.nz;
    R.de(k) = rad2deg(min(max(c(2),-P.de_lim),P.de_lim));
end
end

function s = poles_str(e)
e = sort(e,'ComparisonMethod','real');
parts = cell(1,numel(e));
for i = 1:numel(e)
    if abs(imag(e(i))) < 1e-9
        parts{i} = sprintf('%.3f', real(e(i)));
    else
        parts{i} = sprintf('%.3f%+.3fi', real(e(i)), imag(e(i)));
    end
end
s = strjoin(parts, ', ');
end

function pad_ylim(ax, y)
y = y(isfinite(y)); lo = min(y); hi = max(y); d = hi - lo;
if d < 1e-6, d = max(abs(hi),1)*0.1; end
ylim(ax, [lo - 0.08*d, hi + 0.08*d]);
end

function style_light(fig, tl)
try theme(fig,'light'); catch, end
for a = findall(fig,'Type','axes')'
    set(a,'Color','w','XColor','k','YColor','k','GridColor',[0.4 0.4 0.4],'GridAlpha',0.25);
    a.XLabel.Color = 'k'; a.YLabel.Color = 'k';
    try a.Toolbar.Visible = 'off'; catch, end
end
for l = findall(fig,'Type','legend')'
    set(l,'TextColor','k','Color','w','EdgeColor',[0.7 0.7 0.7]);
end
tl.Title.Color = 'k';
end