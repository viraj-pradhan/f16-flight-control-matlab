% STEP 5: Robustness of the autopilot
% Flies the same climb-and-turn mission (+500 ft, +30 deg heading, 500 ft/s) three ways:
%   1. Ideal       : instant step commands, perfect actuators, no noise, no wind
%   2. Shaped+act  : smoothed commands, actuator lag and rate limits
%   3. Realistic   : shaped + actuators + sensor noise + gusts
%                    (15 ft/s updraft at 40 s, 10 ft/s side gust at 50 s)
% Needs Control System Toolbox. Needs f16_design, f16_fly, f16_metrics and the
% updated f16_dynamics (optional wind input). Saves f16_step5_robustness.png.
clear; clc; close all

P = f16_params();
D = f16_design(P);
fprintf('Pitch SAS: K_w = %.4f, K_q = %.4f | Yaw damper K_r = %.3f, roll damper K_p = %.2f\n', ...
        D.Kw, D.Kq, D.Kr, D.Kp);

cases = {'Ideal (instant commands)',   struct('ctrl','pid');
         'Shaped + actuators',         struct('ctrl','pid','shaping',true,'actuators',true);
         'Realistic (+ noise, gusts)', struct('ctrl','pid','shaping',true,'actuators',true,'noise',true,'gust',true)};
nC = size(cases,1);
Rs = cell(1,nC);  Ms = cell(1,nC);
for i = 1:nC
    fprintf('Running: %s ...\n', cases{i,1});
    Rs{i} = f16_fly(D,P,cases{i,2});
    Ms{i} = f16_metrics(Rs{i});
end

% ---- metrics table ----
fprintf('\n%-30s %12s %12s %12s\n','', 'Ideal','Shaped+act','Realistic');
rows = {'Altitude overshoot (ft)','h_over'; 'Altitude within 25 ft (s)','h_set25'; ...
        'Heading peak (deg)','psi_peak'; 'Heading within 1 deg (s)','psi_set1'; ...
        'Peak bank (deg)','bank_max'; 'Peak sideslip (deg)','beta_max'; ...
        'Elevator min (deg)','de_min'; 'Elevator max (deg)','de_max'; ...
        'Peak aileron (deg)','da_max'; 'Peak rudder (deg)','dr_max'; ...
        'Throttle min','thr_min'; 'Throttle max','thr_max'};
for r = 1:size(rows,1)
    fprintf('%-30s %12.2f %12.2f %12.2f\n', rows{r,1}, Ms{1}.(rows{r,2}), Ms{2}.(rows{r,2}), Ms{3}.(rows{r,2}));
end

% ---- gust effect (after 38 s) ----
fprintf('\nGust effect in the realistic run (after t = 38 s):\n');
for i = 2:3
    R = Rs{i};  ii = R.t >= 38;
    fprintf('  %-28s max |altitude error| %.2f ft, max |heading error| %.2f deg, max |sideslip| %.2f deg\n', ...
        cases{i,1}, max(abs(R.h(ii)-500)), max(abs(rad2deg(R.psi(ii))-30)), max(abs(rad2deg(R.beta(ii)))));
end

% ---- plot ----
fig = figure('Color','w','Position',[100 100 1100 760],'Visible','on');
try theme(fig,'light'); catch, end
tl = tiledlayout(fig,3,2,'TileSpacing','compact');
ax = arrayfun(@(i) nexttile(tl), 1:6);
for i = 1:6, hold(ax(i),'on'); grid(ax(i),'on'); end
col = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
for i = 1:nC
    R = Rs{i};
    plot(ax(1),R.t,R.h,'Color',col(i,:),'LineWidth',1.8);
    plot(ax(2),R.t,rad2deg(R.psi),'Color',col(i,:),'LineWidth',1.8);
    plot(ax(3),R.t,rad2deg(R.de),'Color',col(i,:),'LineWidth',1.5);
    plot(ax(4),R.t,rad2deg(R.da),'Color',col(i,:),'LineWidth',1.5);
    plot(ax(5),R.t,rad2deg(R.dr),'Color',col(i,:),'LineWidth',1.5);
    plot(ax(6),R.t,R.thr,'Color',col(i,:),'LineWidth',1.5);
end
Rc = Rs{1};
plot(ax(1),Rc.t,500*(Rc.t>=2),'k--','LineWidth',1);
plot(ax(2),Rc.t,30*(Rc.t>=2),'k--','LineWidth',1);
ylabel(ax(1),'Altitude (ft)');   ylabel(ax(2),'Heading (deg)');
ylabel(ax(3),'Elevator (deg)');  ylabel(ax(4),'Aileron (deg)');
ylabel(ax(5),'Rudder (deg)');    ylabel(ax(6),'Throttle (0-1)');
xlabel(ax(5),'Time (s)');        xlabel(ax(6),'Time (s)');
for i = 1:6
    xline(ax(i),40,':','Color',[0.4 0.4 0.4],'HandleVisibility','off');
    xline(ax(i),50,':','Color',[0.4 0.4 0.4],'HandleVisibility','off');
end
legend(ax(1),[cases(:,1); {'Command'}],'Location','southeast');
title(tl,'Autopilot robustness: smoothed commands, actuator limits, noise and gusts (dotted: gusts at 40 s and 50 s)');
style_light(fig,tl);
drawnow;
outfile = fullfile(pwd,'f16_step5_robustness.png');
exportgraphics(fig,outfile,'Resolution',200,'BackgroundColor','white');
fprintf('\nSaved: %s\n', outfile);

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