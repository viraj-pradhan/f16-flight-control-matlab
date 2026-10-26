% STEP 6: PID vs LQR (with integral action) autopilot comparison
% Same mission (+500 ft, +30 deg heading, 500 ft/s), same shaped commands, same
% actuators, same autothrottle. Only the altitude and heading control differs:
%   PID : cascaded loops (altitude -> climb rate -> pitch -> elevator; heading -> bank -> aileron)
%         on top of the pitch SAS and the yaw and roll dampers
%   LQI : one LQR design with integral action per axis
%         (longitudinal [w theta q h] + altitude integral; lateral [v phi p r psi] + heading integral)
% Two scenarios: nominal (shaped + actuators) and realistic (+ noise + gusts).
% Needs Control System Toolbox. Saves f16_step6_pid_vs_lqr.png and f16_step6_realistic.png.
% The two designs are hand-tuned by me, so the results compare these tunings,
% not PID and LQR in general.
clear; clc; close all

P = f16_params();
D = f16_design(P);
fprintf('LQI longitudinal poles:\n'); disp(D.poles_lqi_lon)
fprintf('LQI lateral poles:\n');      disp(D.poles_lqi_lat)

nom = struct('shaping',true,'actuators',true);
rea = struct('shaping',true,'actuators',true,'noise',true,'gust',true);
runs = {'PID nominal', 'pid', nom; 'LQI nominal', 'lqi', nom; ...
        'PID realistic','pid', rea; 'LQI realistic','lqi', rea};
Rs = cell(1,4);  Ms = cell(1,4);
for i = 1:4
    fprintf('Running: %s ...\n', runs{i,1});
    c = runs{i,3};  c.ctrl = runs{i,2};
    Rs{i} = f16_fly(D,P,c);
    Ms{i} = f16_metrics(Rs{i});
end

% ---- metrics table ----
fprintf('\n%-34s %12s %12s %14s %14s\n','','PID nominal','LQI nominal','PID realistic','LQI realistic');
rows = {'Altitude overshoot (ft)','h_over'; 'Altitude within 25 ft (s)','h_set25'; ...
        'Heading peak (deg)','psi_peak'; 'Heading within 1 deg (s)','psi_set1'; ...
        'Altitude IAE (ft*s)','IAE_h'; 'Heading IAE (deg*s)','IAE_psi'; ...
        'Peak bank (deg)','bank_max'; 'Peak sideslip (deg)','beta_max'; ...
        'Elevator RMS (deg)','rms_de'; 'Aileron RMS (deg)','rms_da'; 'Rudder RMS (deg)','rms_dr'; ...
        'Max elevator (deg)','de_max'; 'Peak aileron (deg)','da_max'; 'Peak rudder (deg)','dr_max'};
for r = 1:size(rows,1)
    fprintf('%-34s %12.2f %12.2f %14.2f %14.2f\n', rows{r,1}, ...
        Ms{1}.(rows{r,2}), Ms{2}.(rows{r,2}), Ms{3}.(rows{r,2}), Ms{4}.(rows{r,2}));
end
fprintf('\nGust effect in the realistic runs (after t = 38 s):\n');
for i = 3:4
    R = Rs{i};  ii = R.t >= 38;
    fprintf('  %-16s max |altitude error| %.2f ft, max |heading error| %.2f deg, max |sideslip| %.2f deg\n', ...
        runs{i,1}, max(abs(R.h(ii)-500)), max(abs(rad2deg(R.psi(ii))-30)), max(abs(rad2deg(R.beta(ii)))));
end
fprintf('(Noise runs use random numbers, so these values change slightly from machine to machine.)\n');

colP = [0.00 0.45 0.74];  colL = [0.85 0.33 0.10];

% ---- figure 1: nominal ----
fig1 = figure('Color','w','Position',[100 100 1100 760],'Visible','on');
try theme(fig1,'light'); catch, end
tl1 = tiledlayout(fig1,3,2,'TileSpacing','compact');
a1 = arrayfun(@(i) nexttile(tl1), 1:6);
for i = 1:6, hold(a1(i),'on'); grid(a1(i),'on'); end
for k = 1:2
    R = Rs{k};  c = {colP,colL};  c = c{k};
    plot(a1(1),R.t,R.h,'Color',c,'LineWidth',1.8);
    plot(a1(2),R.t,rad2deg(R.psi),'Color',c,'LineWidth',1.8);
    plot(a1(3),R.t,rad2deg(R.de),'Color',c,'LineWidth',1.6);
    plot(a1(4),R.t,rad2deg(R.da),'Color',c,'LineWidth',1.6);
    plot(a1(5),R.t,rad2deg(R.dr),'Color',c,'LineWidth',1.6);
    plot(a1(6),R.t,rad2deg(R.beta),'Color',c,'LineWidth',1.6);
end
R = Rs{1};
plot(a1(1),R.t,500*(R.t>=2),'k--','LineWidth',1);
plot(a1(2),R.t,30*(R.t>=2),'k--','LineWidth',1);
ylabel(a1(1),'Altitude (ft)');   ylabel(a1(2),'Heading (deg)');
ylabel(a1(3),'Elevator (deg)');  ylabel(a1(4),'Aileron (deg)');
ylabel(a1(5),'Rudder (deg)');    ylabel(a1(6),'Sideslip (deg)');
xlabel(a1(5),'Time (s)');        xlabel(a1(6),'Time (s)');
legend(a1(1),{'PID autopilot','LQR + integral (LQI)','Command'},'Location','southeast');
title(tl1,'PID vs LQR autopilot: +500 ft climb and 30 deg turn (nominal)');
style_light(fig1,tl1);
drawnow;
exportgraphics(fig1,fullfile(pwd,'f16_step6_pid_vs_lqr.png'),'Resolution',200,'BackgroundColor','white');

% ---- figure 2: realistic (noise + gusts) ----
fig2 = figure('Color','w','Position',[100 100 1100 640],'Visible','on');
try theme(fig2,'light'); catch, end
tl2 = tiledlayout(fig2,2,2,'TileSpacing','compact');
a2 = arrayfun(@(i) nexttile(tl2), 1:4);
for i = 1:4, hold(a2(i),'on'); grid(a2(i),'on'); end
for k = 3:4
    R = Rs{k};  c = {colP,colL};  c = c{k-2};
    plot(a2(1),R.t,R.h,'Color',c,'LineWidth',1.6);
    plot(a2(2),R.t,rad2deg(R.psi),'Color',c,'LineWidth',1.6);
    plot(a2(3),R.t,rad2deg(R.de),'Color',c,'LineWidth',1.3);
    plot(a2(4),R.t,rad2deg(R.dr),'Color',c,'LineWidth',1.3);
end
ylabel(a2(1),'Altitude (ft)');  ylabel(a2(2),'Heading (deg)');
ylabel(a2(3),'Elevator (deg)'); ylabel(a2(4),'Rudder (deg)');
xlabel(a2(3),'Time (s)');       xlabel(a2(4),'Time (s)');
for i = 1:4
    xline(a2(i),40,':','Color',[0.4 0.4 0.4],'HandleVisibility','off');
    xline(a2(i),50,':','Color',[0.4 0.4 0.4],'HandleVisibility','off');
end
legend(a2(1),{'PID autopilot','LQR + integral (LQI)'},'Location','southeast');
title(tl2,'PID vs LQR with sensor noise and gusts (dotted: updraft at 40 s, side gust at 50 s)');
style_light(fig2,tl2);
drawnow;
exportgraphics(fig2,fullfile(pwd,'f16_step6_realistic.png'),'Resolution',200,'BackgroundColor','white');
fprintf('\nSaved f16_step6_pid_vs_lqr.png and f16_step6_realistic.png in %s\n', pwd);

% ---------- local function ----------
function style_light(fig,tl)
try theme(fig,'light'); catch, end
for a = findall(fig,'Type','axes')'
    set(a,'Color','w','XColor','k','YColor','k','GridColor',[0.4 0.4 0.4],'GridAlpha',0.25);
    a.XLabel.Color = 'k';  a.YLabel.Color = 'k';
    try a.Toolbar.Visible = 'off'; catch, end     % keep the axes toolbar out of the saved PNG
end
for l = findall(fig,'Type','legend')'
    set(l,'TextColor','k','Color','w','EdgeColor',[0.7 0.7 0.7]);
end
tl.Title.Color = 'k';
end