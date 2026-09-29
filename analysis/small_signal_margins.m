%SMALL_SIGNAL_MARGINS  Stability margins of the small-signal loops at the plant input.
%
%   For state feedback u = K*X on the linearised plant, Eq. (13), the loop broken
%   at the plant input is L(s) = -K (sI - A)^-1 B and the closed loop is A + B*K.
%   Margins are reported for
%     RS-HAC  closed-form gain (proof of Proposition 3), rshac_gain.m
%     LQR     Eq. (25)
%     FC      straight-line inference of Fig. 7 with the fuzzy branch's
%             de-semantization block. The FC gain is computed from evalfis, and
%             the pre-check below confirms the implemented rule-base mapping.
%
%   The open-loop plant has one unstable pole, so gain margins are two-sided.
%   The delay margins are the analytic predictions to compare with experiment E5.
%
%   Run:  >> small_signal_margins          Log: logs/small_signal_margins.txt

clear; clc;
here   = fileparts(mfilename('fullpath'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'small_signal_margins.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('small_signal_margins   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

P = rshac_params();

%% ---- FC pre-check: what does the rule base return? ---------------------------------
fisFile = fullfile(here, '..', 'simulation', 'fuzzy_ctrl_v2.fis');
fis = readfis(fisFile);
sg  = (0.25:0.01:0.75).';
out = evalfis(fis, [sg, 0.5*ones(numel(sg), 3)]);
fprintf('=== FC pre-check: evalfis(fuzzy_ctrl_v2.fis), input x swept, other inputs neutral ===\n');
fprintf('  max|out_x - s| = %.3e    max|out_x - 4s| = %.3e    other outputs: max|out - 0.5| = %.3e\n', ...
        max(abs(out(:, 1) - sg)), max(abs(out(:, 1) - 4*sg)), max(max(abs(out(:, 2:4) - 0.5))));
uFC = 4*P.u_max*out(:, 1) + 2*P.u_min;
fprintf('  u_x(FC) at s = 0.25 / 0.50 / 0.75 : %.3f  %.3f  %.3f m/s^2\n', uFC(1), uFC(sg == 0.5), uFC(end));
if max(abs(out(:, 1) - sg)) < 1e-9
    fprintf('  -> FIS output equals the semantic input in-range; FC line u = u_max(4s - 2)\n\n');
else
    fprintf('  -> FIS output differs from the expected in-range mapping; inspect the FIS before interpreting FC results\n\n');
end

%% ---- small-signal gains -------------------------------------------------------------
[K_RS, ~, ds] = rshac_gain(P);
K_LQR = lqr_gain();                                % Eq. (25), convention u = +K*X
h = 1e-6;  K_FC = zeros(1, 4);
for i = 1:4
    sp = 0.5*ones(1, 4);  sm = sp;
    sp(i) = 0.5 + h;  sm(i) = 0.5 - h;
    op = evalfis(fis, sp);  om = evalfis(fis, sm);
    K_FC(i) = 0.25*4*P.u_max*(op(i) - om(i))/(2*h)*ds(i);
end
names = {'RS-HAC', 'LQR (Eq. 25)', 'FC (Fig. 7 line)'};
Ks    = {K_RS, K_LQR, K_FC};

fprintf('=== small-signal gains (u = +K*X) ===\n');
for m = 1:3
    fprintf('  %-17s K = %s\n', names{m}, mat2str(Ks{m}, 5));
end
fprintf('  ratios to LQR:  RS-HAC %s   FC %s\n\n', mat2str(K_RS./K_LQR, 3), mat2str(K_FC./K_LQR, 3));

%% ---- closed-loop poles and loop margins -----------------------------------------------
Gd = c2d(ss(P.A, P.B, eye(4), zeros(4, 1)), P.Ts, 'zoh');
R  = struct();
fprintf('=== closed-loop poles (linearised plant, continuous time) ===\n');
for m = 1:3
    K   = Ks{m};
    ev  = eig(P.A + P.B*K);
    [~, ix] = sort(abs(real(ev)));  ev = ev(ix);
    zeta = -real(ev)./abs(ev);
    fprintf('  %-17s poles %s\n', names{m}, mat2str(ev.', 4));
    fprintf('  %-17s zeta  %s   (dominant pair: Re = %.3f, zeta = %.3f)\n', '', mat2str(zeta.', 3), real(ev(1)), zeta(1));
    assert(all(real(ev) < 0), '%s: closed loop not Hurwitz', names{m});
    R(m).name = names{m};  R(m).K = K;  R(m).poles = ev;  R(m).zeta = zeta;
end

fprintf('\n=== loop margins at the plant input, L(s) = -K(sI-A)^-1 B ===\n');
fprintf('  %-17s %12s %12s %14s %12s %12s %16s\n', 'controller', 'GM low [dB]', 'GM high [dB]', 'PM [deg]', 'DM [ms]', 'DM_zoh [ms]', 'disk GM [dB]');
for m = 1:3
    K  = Ks{m};
    L  = ss(P.A, P.B, -K, 0);
    S  = allmargin(L);
    assert(S.Stable == 1, '%s: allmargin reports an unstable closed loop', names{m});
    gm = S.GainMargin;
    gmLow  = max([gm(gm < 1), 0]);                   % largest admissible gain reduction
    gmHigh = min([gm(gm > 1), Inf]);                 % largest admissible gain increase
    pm = min(abs(S.PhaseMargin));
    dm = min(S.DelayMargin);

    Ld = ss(Gd.A, Gd.B, -K, 0, P.Ts);
    Sd = allmargin(Ld);
    evd = eig(Gd.A + Gd.B*K);
    assert(all(abs(evd) < 1), '%s: sampled closed loop not Schur stable', names{m});
    dmd = min(Sd.DelayMargin)*P.Ts;                  % discrete delay margin is in samples

    DM = diskmargin(L);
    fprintf('  %-17s %12.2f %12.2f %14.2f %12.2f %12.2f %16s\n', names{m}, 20*log10(gmLow), 20*log10(gmHigh), ...
            pm, 1e3*dm, 1e3*dmd, mat2str(round(20*log10(DM.GainMargin), 2)));
    R(m).GM = gm;  R(m).PM = S.PhaseMargin;  R(m).DM = S.DelayMargin;  R(m).DMzoh = dmd;
    R(m).rhoZoh = max(abs(evd));  R(m).disk = DM;
end
fprintf('  (GM low: gain reduction the loop tolerates; DM_zoh: sampled at T_s = %g ms with zero-order hold)\n', 1e3*P.Ts);

save(fullfile(logDir, 'small_signal_margins.mat'), 'R');
fprintf('\nSaved logs/small_signal_margins.mat\n');
diary off;
