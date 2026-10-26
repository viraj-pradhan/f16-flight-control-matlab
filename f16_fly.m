function R = f16_fly(D, P, cfg)
% Closed-loop simulation of the climb-and-turn mission:
%   at t = 2 s command +500 ft altitude and +30 deg heading, hold 500 ft/s.
%
% cfg fields (all optional):
%   ctrl      'pid' (default) or 'lqi'
%   shaping   true = smooth the altitude and heading commands
%   actuators true = first-order actuators with position and rate limits
%   noise     true = sensor noise (with light measurement filtering)
%   gust      true = vertical gust at t = 40 s and lateral gust at t = 50 s
%   tf, dt    final time (default 60 s) and fixed step (default 0.01 s, RK4)
%   seed      random seed for the noise (default 1)
%
% Both controllers sit on top of the same pitch SAS / lateral dampers where
% noted, use the same autothrottle, and see the same shaped commands, so the
% comparison in step 6 is fair.

def = struct('ctrl','pid','shaping',false,'actuators',false,'noise',false, ...
             'gust',false,'tf',60,'dt',0.01,'seed',1);
fn = fieldnames(def);
for i = 1:numel(fn)
    if ~isfield(cfg,fn{i}), cfg.(fn{i}) = def.(fn{i}); end
end
dt = cfg.dt;  n = round(cfg.tf/dt) + 1;
g  = D.G;  x = D.x0;  u0 = D.u0;  x00 = D.x0;
th0 = x00(5);  w0 = x00(3);  de0 = u0(2);  thr0 = u0(1);  V0 = D.V0;
Hc = 500;  PSc = deg2rad(30);  tcmd = 2;
if cfg.noise, rng(cfg.seed); end

% shaping, actuator and noise settings
tau_h = 3.0;  psi_rate = deg2rad(1.8);  tau_psi = 1.5;
tau_act = 0.05;  rate_lim = deg2rad(60);
lim = [P.de_lim; P.da_lim; P.dr_lim];
sig.pqr = deg2rad(0.2);  sig.att = deg2rad(0.1);  sig.psi = deg2rad(0.2);
sig.h = 3.0;  sig.V = 1.0;  sig.w = 0.9;  sig.hd = 0.5;

% controller and filter states
act = [de0; 0; 0];
I_c = 0; I_th = 0; I_v = 0; xw = 0; zh = 0; zp = 0;
Vf = V0; hdf = 0; hf = 0; psf = 0; psr = 0;

R.X = zeros(n,13);
names = {'t','h','psi','V','phi','th','beta','alpha','p','q','r','de','da','dr','thr', ...
         'de_c','da_c','dr_c','hc','psic','ww','vw'};
for i = 1:numel(names), R.(names{i}) = zeros(n,1); end

for k = 1:n
    t = (k-1)*dt;

    % ---- commands (optionally shaped) ----
    if t >= tcmd, hc = Hc; pc = PSc; else, hc = 0; pc = 0; end
    if cfg.shaping
        hf  = hf + dt*(hc - hf)/tau_h;
        psr = psr + clip(pc - psr, -psi_rate*dt, psi_rate*dt);
        psf = psf + dt*(psr - psf)/tau_psi;
        hcmd = hf;  pcmd = psf;
    else
        hcmd = hc;  pcmd = pc;
    end

    % ---- measurements ----
    ub = x(1); vb = x(2); wb = x(3); phi = x(4); th = x(5); psi = x(6);
    p = x(7); q = x(8); r = x(9); h = x(12);
    Vt = sqrt(ub^2 + vb^2 + wb^2);
    hdot = ub*sin(th) - vb*sin(phi)*cos(th) - wb*cos(phi)*cos(th);
    m = struct('w',wb,'phi',phi,'th',th,'psi',psi,'p',p,'q',q,'r',r,'h',h,'V',Vt,'hd',hdot);
    if cfg.noise
        m.w   = m.w   + sig.w*randn;     m.phi = m.phi + sig.att*randn;
        m.th  = m.th  + sig.att*randn;   m.psi = m.psi + sig.psi*randn;
        m.p   = m.p   + sig.pqr*randn;   m.q   = m.q   + sig.pqr*randn;
        m.r   = m.r   + sig.pqr*randn;   m.h   = m.h   + sig.h*randn;
        m.V   = m.V   + sig.V*randn;     m.hd  = m.hd  + sig.hd*randn;
        Vf  = Vf  + dt*(m.V  - Vf)/1.0;   hdf = hdf + dt*(m.hd - hdf)/0.3;
        m.V = Vf;  m.hd = hdf;
    end

    % ---- autopilot ----
    if strcmp(cfg.ctrl,'pid')
        hdot_cmd = clip(g.Kh*(hcmd - m.h), -50, 50);
        e_c  = hdot_cmd - m.hd;
        I_c  = clip(I_c + e_c*dt, -30, 30);
        th_cmd = clip(th0 + g.Kc*e_c + g.Kci*I_c, th0 - deg2rad(10), th0 + deg2rad(15));
        e_th = th_cmd - m.th;
        I_th = clip(I_th + e_th*dt, -0.3, 0.3);
        de_c = de0 - (D.Kw*(m.w - w0) + D.Kq*m.q) - (g.Kth*e_th + g.Kthi*I_th);

        phi_cmd = clip(g.Kpsi*(pcmd - m.psi), -deg2rad(30), deg2rad(30));
        da_c = g.Kphi*(phi_cmd - m.phi) - D.Kp*m.p;
        xw   = xw + dt*(m.r - xw)/D.Tw;
        dr_c = D.Kr*(m.r - xw);
    else
        zh = clip(zh + dt*(hcmd - m.h), -3000, 3000);
        xl = [m.w - w0; m.th - th0; m.q; m.h];
        de_c = de0 - (D.KL(1:4)*xl + D.KL(5)*zh);

        zp = clip(zp + dt*(pcmd - m.psi), -3, 3);
        xt = [vb; m.phi; m.p; m.r; m.psi];
        ut = -(D.KT(:,1:5)*xt + D.KT(:,6)*zp);
        da_c = ut(1);  dr_c = ut(2);
    end

    % ---- autothrottle (same for both) ----
    e_v  = V0 - m.V;
    I_v  = clip(I_v + e_v*dt, -40, 40);
    thr_c = clip(thr0 + g.Kv*e_v + g.Kvi*I_v, 0, 1);

    % ---- actuators ----
    cmd = [de_c; da_c; dr_c];
    if cfg.actuators
        rate = min(max((cmd - act)/tau_act, -rate_lim), rate_lim);
        act  = act + dt*rate;
    else
        act = cmd;
    end
    act  = min(max(act, -lim), lim);
    ctrl = [thr_c; act(1); act(2); act(3)];

    % ---- wind ----
    ww = 0; vw = 0;
    if cfg.gust
        if t >= 40 && t < 43, ww = -(15/2)*(1 - cos(2*pi*(t-40)/3)); end   % updraft
        if t >= 50 && t < 53, vw =  (10/2)*(1 - cos(2*pi*(t-50)/3)); end   % side gust
    end
    wind = [0 vw ww];

    % ---- log ----
    R.t(k) = t;  R.h(k) = h;  R.psi(k) = psi;  R.V(k) = Vt;  R.phi(k) = phi;  R.th(k) = th;
    Vair = sqrt(ub^2 + (vb-vw)^2 + (wb-ww)^2);
    R.beta(k)  = asin((vb-vw)/Vair);  R.alpha(k) = atan2(wb-ww,ub);   % air-relative
    R.p(k) = p;  R.q(k) = q;  R.r(k) = r;
    R.de(k) = act(1);  R.da(k) = act(2);  R.dr(k) = act(3);  R.thr(k) = thr_c;
    R.de_c(k) = de_c;  R.da_c(k) = da_c;  R.dr_c(k) = dr_c;
    R.hc(k) = hcmd;  R.psic(k) = pcmd;  R.ww(k) = ww;  R.vw(k) = vw;
    R.X(k,:) = x';

    % ---- RK4 step ----
    k1 = f16_dynamics(x,           ctrl, P, wind);
    k2 = f16_dynamics(x+0.5*dt*k1, ctrl, P, wind);
    k3 = f16_dynamics(x+0.5*dt*k2, ctrl, P, wind);
    k4 = f16_dynamics(x+dt*k3,     ctrl, P, wind);
    x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
R.cfg = cfg;
end

function y = clip(x,lo,hi)
y = min(max(x,lo),hi);
end