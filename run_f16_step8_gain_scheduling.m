% STEP 8: Gain scheduling of the pitch SAS across speed and altitude
% Designs an LQR pitch SAS at 15 trim points (5 speeds x 3 altitudes), stores
% the gains as tables, and compares a SINGLE fixed gain set (the 500 ft/s,
% sea-level design from step 2) against the scheduled gains.
% Needs Control System Toolbox (lqr). Saves two PNGs and f16_gain_schedule.mat.
% Also needs the updated f16_trim.m (better starting guess for other conditions).
clear; clc; close all

P  = f16_params();
Vg = [400 500 600 700 800];            % speed grid (ft/s)
hg = [0 10000 20000];                  % altitude grid (ft)
R  = 1/deg2rad(10)^2;
sp = [3 8];                            % short-period states: w, q

Ka = zeros(numel(Vg),numel(hg));  Kq = Ka;  De0 = Ka;
fprintf('Designing LQR gains at %d trim points...\n', numel(Vg)*numel(hg));
for i = 1:numel(Vg)
    for j = 1:numel(hg)
        [x0,u0,info] = f16_trim(Vg(i),hg(j),P);
        [A,B] = numjac_f16(x0,u0,P);
        Q = diag([1/(Vg(i)*deg2rad(5))^2, 1/deg2rad(30)^2]);
        K = lqr(A(sp,sp), B(sp,2), Q, R);
        Ka(i,j)  = K(1)*Vg(i);         % gain on alpha (rad per rad), smoother than K_w
        Kq(i,j)  = K(2);               % gain on pitch rate
        De0(i,j) = u0(2);              % trim elevator
        if info.residual > 1e-6
            warning('Trim residual %.1e at V=%d, h=%d', info.residual, Vg(i), hg(j));
        end
    end
end

fprintf('\nK_alpha table (rows = V %s, columns = h %s):\n', mat2str(Vg), mat2str(hg));
disp(Ka)
fprintf('K_q table:\n'); disp(Kq)

Fka = griddedInterpolant({Vg,hg}, Ka,  'linear');
Fkq = griddedInterpolant({Vg,hg}, Kq,  'linear');
save('f16_gain_schedule.mat','Vg','hg','Ka','Kq','De0');

% ---- fixed gains = the step 2 design (500 ft/s, sea level) ----
Kw_fix = Ka(Vg==500,hg==0)/500;
Kq_fix = Kq(Vg==500,hg==0);
fprintf('Fixed gains (step 2 design): K_w = %.4f, K_q = %.4f\n', Kw_fix, Kq_fix);

% ---- short-period damping across the envelope ----
Vs = 400:50:800;  hs = [0 10000 20000];
zf = zeros(numel(Vs),numel(hs));  zs = zf;
for j = 1:numel(hs)
    for i = 1:numel(Vs)
        [x0,u0] = f16_trim(Vs(i),hs(j),P);
        [A,B] = numjac_f16(x0,u0,P);
        Asp = A(sp,sp);  Bsp = B(sp,2);
        zf(i,j) = zeta_of(Asp - Bsp*[Kw_fix, Kq_fix]);
        Ks = [Fka(Vs(i),hs(j))/Vs(i), Fkq(Vs(i),hs(j))];
        zs(i,j) = zeta_of(Asp - Bsp*Ks);
    end
end
fprintf('\nShort-period damping ratio across the envelope:\n');
fprintf('  fixed gains    : min %.2f, max %.2f\n', min(zf(:)), max(zf(:)));
fprintf('  scheduled gains: min %.2f, max %.2f\n', min(zs(:)), max(zs(:)));

fig1 = figure('Color','w','Position',[100 100 900 520],'Visible','on');
try theme(fig1,'light'); catch, end
ax = axes(fig1); hold(ax,'on'); grid(ax,'on');
cols = lines(numel(hs));
for j = 1:numel(hs)
    plot(ax,Vs,zs(:,j),'-o','Color',cols(j,:),'LineWidth',1.8,'DisplayName',sprintf('Scheduled, %d ft',hs(j)));
    plot(ax,Vs,zf(:,j),'--s','Color',cols(j,:),'LineWidth',1.2,'DisplayName',sprintf('Fixed, %d ft',hs(j)));
end
xlabel(ax,'Airspeed (ft/s)'); ylabel(ax,'Short-period damping ratio');
title(ax,'Pitch SAS damping across the flight envelope: fixed vs scheduled gains');
legend(ax,'Location','southeast','NumColumns',2);
style_light(fig1);
drawnow;
exportgraphics(fig1,fullfile(pwd,'f16_step8_damping.png'),'Resolution',200,'BackgroundColor','white');

% ---- nonlinear elevator-doublet tests at three flight conditions ----
tests = [400 20000; 800 0; 650 15000];
fig2 = figure('Color','w','Position',[100 100 1100 600],'Visible','on');
try theme(fig2,'light'); catch, end
tl = tiledlayout(fig2,2,3,'TileSpacing','compact');
axA = gobjects(1,3); axE = gobjects(1,3);
for k = 1:3
    axA(k) = nexttile(tl,k);     hold(axA(k),'on'); grid(axA(k),'on');
    axE(k) = nexttile(tl,k+3);   hold(axE(k),'on'); grid(axE(k),'on');
    V = tests(k,1);  h = tests(k,2);
    Ks = [Fka(V,h)/V, Fkq(V,h)];
    r1 = simulate_case(V,h,[Kw_fix Kq_fix],P);
    r2 = simulate_case(V,h,Ks,P);
    plot(axA(k),r1.t,r1.da,'--','LineWidth',1.6);  plot(axA(k),r2.t,r2.da,'-','LineWidth',1.8);
    plot(axE(k),r1.t,r1.de,'--','LineWidth',1.6);  plot(axE(k),r2.t,r2.de,'-','LineWidth',1.8);
    title(axA(k),sprintf('%d ft/s, %d ft',V,h));
    xlabel(axE(k),'Time (s)');
    fprintf('V=%d h=%d: peak d_alpha fixed %.2f deg, scheduled %.2f deg\n',V,h,max(abs(r1.da)),max(abs(r2.da)));
end
ylabel(axA(1),'\Delta\alpha (deg)'); ylabel(axE(1),'Elevator (deg)');
legend(axA(1),{'Fixed gains','Scheduled gains'},'Location','best');
title(tl,'Elevator doublet with SAS on: fixed vs scheduled gains');
style_light(fig2,tl);
drawnow;
exportgraphics(fig2,fullfile(pwd,'f16_step8_doublets.png'),'Resolution',200,'BackgroundColor','white');
fprintf('\nSaved f16_step8_damping.png, f16_step8_doublets.png and f16_gain_schedule.mat in %s\n',pwd);

% ---------- local functions ----------
function z = zeta_of(M)
e = eig(M);
if any(real(e) >= 0), z = NaN; return; end          % NaN means unstable
if any(abs(imag(e)) > 1e-9)
    e1 = e(imag(e) > 0);  z = -real(e1(1))/abs(e1(1));
else
    z = 1;
end
end

function r = simulate_case(V,h,Kgain,P)
[x0,u0] = f16_trim(V,h,P);
amp = deg2rad(1.5);
dd  = @(t) -amp*(t>=1 & t<1.5) + amp*(t>=1.5 & t<2);
ctrl = @(t,x) [u0(1); u0(2) + dd(t) - Kgain*[x(3)-x0(3); x(8)-x0(8)]; 0; 0];
opts = odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',0.02);
[t,X] = ode45(@(t,x) f16_dynamics(x,ctrl(t,x),P),[0 5],x0,opts);
r.t = t;  r.da = zeros(numel(t),1);  r.de = r.da;
for k = 1:numel(t)
    c = ctrl(t(k),X(k,:)');
    [~,o] = f16_dynamics(X(k,:)',c,P);
    r.da(k) = rad2deg(o.alpha - atan2(x0(3),x0(1)));
    r.de(k) = rad2deg(min(max(c(2),-P.de_lim),P.de_lim));
end
end

function style_light(fig,tl)
try theme(fig,'light'); catch, end
for a = findall(fig,'Type','axes')'
    set(a,'Color','w','XColor','k','YColor','k','GridColor',[0.4 0.4 0.4],'GridAlpha',0.25);
    a.XLabel.Color = 'k';  a.YLabel.Color = 'k';
    a.Title.Color = 'k';
    try a.Toolbar.Visible = 'off'; catch, end     % keep the axes toolbar out of the saved PNG
end
for l = findall(fig,'Type','legend')'
    set(l,'TextColor','k','Color','w','EdgeColor',[0.7 0.7 0.7]);
end
if nargin > 1, tl.Title.Color = 'k'; end
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