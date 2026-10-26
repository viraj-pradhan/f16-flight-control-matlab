function M = f16_metrics(R)
% Performance numbers for one run of f16_fly (mission: +500 ft, +30 deg at t = 2 s).
t = R.t;  t_after = 2;
h = R.h;  psi = rad2deg(R.psi);
i2 = t >= t_after;

M.h_over     = max(h) - 500;                          % altitude overshoot (ft)
M.h_set25    = settle(t, h, 500, 25, t_after);        % time to stay within 25 ft (s)
M.psi_peak   = max(psi);                              % heading peak (deg)
M.psi_set1   = settle(t, psi, 30, 1, t_after);        % time to stay within 1 deg (s)
M.V_min      = min(R.V);   M.V_max = max(R.V);
M.bank_max   = max(rad2deg(R.phi));
M.pitch_max  = max(rad2deg(R.th));
M.beta_max   = max(abs(rad2deg(R.beta)));
M.de_min     = min(rad2deg(R.de));    M.de_max = max(rad2deg(R.de));
M.da_max     = max(abs(rad2deg(R.da)));
M.dr_max     = max(abs(rad2deg(R.dr)));
M.thr_min    = min(R.thr);  M.thr_max = max(R.thr);

% tracking error and control effort against the RAW commands (+500 ft, +30 deg)
M.IAE_h   = trapz(t(i2), abs(500 - h(i2)));           % ft*s
M.IAE_psi = trapz(t(i2), abs(30 - psi(i2)));          % deg*s
de0 = R.de(1);
M.rms_de = sqrt(mean(rad2deg(R.de(i2) - de0).^2));    % deg
M.rms_da = sqrt(mean(rad2deg(R.da(i2)).^2));
M.rms_dr = sqrt(mean(rad2deg(R.dr(i2)).^2));
end

function ts = settle(t, sig, target, band, t_after)
idx = find(abs(sig - target) > band, 1, 'last');
if isempty(idx), ts = 0; else, ts = t(idx) - t_after; end
end