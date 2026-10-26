% HOW IT GETS ITS DATA
%   By default it flies the climb-and-turn mission itself with f16_fly (PID
%   autopilot, smoothed commands, actuator limits) and plays that flight back.
%   Set ctrl = 'lqi' to play the LQR autopilot instead.
%   Optional: set useLog = true to play a saved log instead. The log file
%   f16_step4_log.mat must contain t (N x 1) and X (N x 13 states in this
%   model's order [u v w phi theta psi p q r pn pe h pow]).
%
% Needs the files from steps 1 to 6 in the same folder (f16_design, f16_fly).
clear; clc; close all

% ---------------- settings ----------------
camMode   = 'chase';     % 'chase' (follows the jet) or 'overview' (fixed camera)
speedUp   = 2;           % 2 = video plays twice as fast as the simulation
fps       = 30;
modelScale = 25;         % jet is drawn 25x larger so it is visible at this scale
makeVideo = true;
videoName = 'f16_step7_playback.mp4';
% ------------------------------------------

ctrl   = 'pid';          % 'pid' or 'lqi'
useLog = false;          % true = play f16_step4_log.mat instead of flying the mission

logfile = 'f16_step4_log.mat';
if useLog && isfile(logfile)
    S = load(logfile);  t = S.t(:);  X = S.X;
    fprintf('Playing back saved log: %s\n', logfile);
else
    P = f16_params();  D = f16_design(P);
    fprintf('Flying the mission with the %s autopilot ...\n', upper(ctrl));
    Rm = f16_fly(D, P, struct('ctrl',ctrl,'shaping',true,'actuators',true));
    t = Rm.t;  X = Rm.X;
end

% ---- resample to video frames ----
nFrames = max(2, round(t(end)/speedUp*fps));
tk = linspace(t(1), t(end), nFrames)';
Xk = interp1(t, X, tk, 'linear');
phi = Xk(:,4); th = Xk(:,5); psi = unwrap(Xk(:,6));
pn  = Xk(:,10); pe = Xk(:,11); h = Xk(:,12);
spd = sqrt(sum(Xk(:,1:3).^2,2));
E = pe;  N = pn;  U = h;                       % display axes: East, North, Up

% ---- jet geometry in body axes (x forward, y right, z down), feet ----
body = {};  cols = {};
nose = [28 0 0];  rl = [-22 -3 0];  rr = [-22 3 0];  top = [-5 0 -3];  bot = [-5 0 2.5];
body{end+1} = [nose; rl; top];   cols{end+1} = [0.55 0.58 0.62];
body{end+1} = [nose; top; rr];   cols{end+1} = [0.60 0.63 0.67];
body{end+1} = [nose; rr; bot];   cols{end+1} = [0.45 0.48 0.52];
body{end+1} = [nose; bot; rl];   cols{end+1} = [0.42 0.45 0.49];
body{end+1} = [rl; rr; top];     cols{end+1} = [0.50 0.53 0.57];
body{end+1} = [rl; rr; bot];     cols{end+1} = [0.38 0.40 0.44];
body{end+1} = [8 -2.5 0.3; -14 -15 0.3; -14 -2.5 0.3];   cols{end+1} = [0.20 0.35 0.65];   % left wing
body{end+1} = [8  2.5 0.3; -14  15 0.3; -14  2.5 0.3];   cols{end+1} = [0.20 0.35 0.65];   % right wing
body{end+1} = [-12 0 -2.5; -22 0 -10; -22 0 -2.5];       cols{end+1} = [0.75 0.20 0.20];   % tail fin
body{end+1} = [-17 -2.5 0.3; -22 -8 0.3; -22 -2.5 0.3];  cols{end+1} = [0.20 0.35 0.65];
body{end+1} = [-17  2.5 0.3; -22  8 0.3; -22  2.5 0.3];  cols{end+1} = [0.20 0.35 0.65];

% ---- scene ----
fig = figure('Color','w','Position',[100 100 1280 720],'Visible','on');
try theme(fig,'light'); catch, end
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
set(ax,'Color',[0.80 0.90 1.00],'XColor','k','YColor','k','ZColor','k','GridColor',[0.3 0.3 0.3],'GridAlpha',0.2);
xlabel(ax,'East (ft)'); ylabel(ax,'North (ft)'); zlabel(ax,'Altitude (ft)');
pad = 4000;
xl = [min(E)-pad, max(E)+pad];  yl = [min(N)-pad, max(N)+pad];
zl = [-200, max(U)+1500];
xlim(ax,xl); ylim(ax,yl); zlim(ax,zl);
patch(ax,[xl(1) xl(2) xl(2) xl(1)],[yl(1) yl(1) yl(2) yl(2)],[-120 -120 -120 -120],[0.55 0.75 0.50], ...
      'FaceAlpha',0.55,'EdgeColor','none');                       % ground
trailAll = plot3(ax,E,N,U,'-','Color',[0.7 0.7 0.7],'LineWidth',1);   % full path, faint
trail    = plot3(ax,E(1),N(1),U(1),'-','Color',[0.85 0.33 0.10],'LineWidth',2.5);
hp = gobjects(1,numel(body));
for k = 1:numel(body)
    hp(k) = patch(ax,'Vertices',body{k},'Faces',[1 2 3],'FaceColor',cols{k},'EdgeColor','k','LineWidth',0.5);
end
ttl = title(ax,'','Color','k','FontSize',14);
camproj(ax,'perspective'); camva(ax,30); camup(ax,[0 0 1]);
if strcmp(camMode,'overview')
    campos(ax,[mean(xl)+0.9*diff(xl), mean(yl)-1.3*diff(yl), max(zl)*6+3000]);
    camtarget(ax,[mean(xl) mean(yl) mean(zl)]);
end

% ---- video writer ----
if makeVideo
    vw = VideoWriter(videoName,'MPEG-4');  vw.FrameRate = fps;  open(vw);
end

% smoothed heading for a steady chase camera
psiS = movmean(psi, max(5,round(fps/2)));

for n = 1:nFrames
    % body -> NED rotation (Rz*Ry*Rx), same convention as the model
    cph = cos(phi(n)); sph = sin(phi(n)); cth = cos(th(n)); sth = sin(th(n));
    cps = cos(psi(n)); sps = sin(psi(n));
    Rx = [1 0 0; 0 cph -sph; 0 sph cph];
    Ry = [cth 0 sth; 0 1 0; -sth 0 cth];
    Rz = [cps -sps 0; sps cps 0; 0 0 1];
    R  = Rz*Ry*Rx;
    for k = 1:numel(body)
        ned = (R*(modelScale*body{k})')';                 % rows: [north east down] offsets
        enu = [ned(:,2) + E(n), ned(:,1) + N(n), -ned(:,3) + U(n)];
        set(hp(k),'Vertices',enu);
    end
    set(trail,'XData',E(1:n),'YData',N(1:n),'ZData',U(1:n));

    if strcmp(camMode,'chase')
        fwd = [sin(psiS(n)) cos(psiS(n)) 0];
        pos = [E(n) N(n) U(n)];
        campos(ax, pos - 1800*fwd + [0 0 500]);
        camtarget(ax, pos + 600*fwd);
    end

    hdg = mod(rad2deg(psi(n)),360);
    set(ttl,'String',sprintf('t = %5.1f s    altitude = %5.0f ft    heading = %5.1f deg    bank = %5.1f deg    speed = %4.0f ft/s', ...
        tk(n), U(n), hdg, rad2deg(phi(n)), spd(n)));
    drawnow;
    if makeVideo, writeVideo(vw,getframe(fig)); end
end
if makeVideo
    close(vw);
    fprintf('Saved video: %s\n', fullfile(pwd,videoName));
end