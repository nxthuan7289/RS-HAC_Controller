%EXP_E4_SATURATION  Experiment E4: actuator-bound sweep at a FIXED control law.
%
%   Protocol:
%     - the control law and the nominal de-semantization scale of Eq. (19) are HELD
%       FIXED at u_max = 3 g; only the actuator saturation block is swept. Changing
%       both would change the controller gain, which is a redesign study, not a
%       robustness study, and is not what is reported here.
%     - grid u_sat/g in {0.05, 0.1, 0.15, 0.25, 0.5, 0.75, 1, 1.5, 2, 3}, extended below
%       the {0.5 ... 3} of the original protocol so that the threshold is bracketed for
%       every controller, then a bisection between the
%       largest failing and the smallest succeeding bound, to 0.01 g.
%     - deterministic, no noise, one run per level; horizon 10 s; success = inside the
%       band of Eq. (26) for the last 1 s, |x| <= 0.43 m and |q| < 90 deg throughout.
%     - plant: Eq. (12) nominal (cartpole_nl), not the Simulink plant, which carries
%       the extra kdr, kt, and Fc terms. These thresholds are not directly comparable
%       with the main Simulink tables.
%
%   Run:  >> exp_e4_saturation          Log: logs/exp_e4_saturation.txt
%   (run from experiments/; the script puts ../analysis on the path itself)

clear; clc;
here   = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'analysis'));          % rshac_params, rshac_law, cartpole_nl
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'exp_e4_saturation.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('exp_e4_saturation   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

q0deg = [10 20 30];
[P, C] = exp_setup(q0deg);
g     = P.g;
grid4 = [0.05 0.1 0.15 0.25 0.5 0.75 1 1.5 2 3];   % in units of g
T     = 10;  dwell = 1;
fprintf('horizon %g s, dwell %g s, T_s = %g ms, Eq. (19) scale u_max = %.4f m/s^2 (FIXED)\n\n', ...
        T, dwell, 1e3*P.Ts, P.u_max);

R = struct([]);  n = 0;
for c = 1:numel(C)
  for s = 1:numel(q0deg)
    X0 = [0; 0; deg2rad(q0deg(s)); 0];
    fprintf('=== %s,  q0 = %d deg ===\n', C{c}.name, q0deg(s));
    fprintf('  %9s %4s %8s %9s %8s %8s %9s  %s\n', ...
            'u_sat', 'ok', 'tEnter', 'tSettle', 'satFrac', 'max|x|', 'Su', 'note');
    okv = false(1, numel(grid4));
    for j = 1:numel(grid4)
      M = simulate_fixed_ctrl(C{c}.fun, X0, P, struct('uSat', grid4(j)*g, 'dMs', 0, ...
                                                      'T', T, 'dwell', dwell));
      okv(j) = M.success;
      fprintf('  %6.2f=%.2fg %4d %8.3f %9.3f %8.3f %8.3f %9.3f  %s\n', grid4(j)*g, grid4(j), ...
              M.success, M.tEnter, M.tSettle, M.satFrac, M.Dx, M.Su, M.why);
      n = n + 1;
      R(n).ctrl = C{c}.name;  R(n).q0 = q0deg(s);  R(n).uSat = grid4(j)*g;  R(n).M = M;
    end

    % largest bound that still FAILS, below the top contiguous block of successes
    top = numel(grid4);
    while top >= 1 && okv(top), top = top - 1; end
    if top == numel(grid4)
      fprintf('  -> %s, q0=%d deg: FAILS even at the nominal %g g\n\n', C{c}.name, q0deg(s), grid4(end));
      continue
    end
    if top == 0
      fprintf('  -> %s, q0=%d deg: still stabilizes at the smallest bound tried, %.2f g (%.2f m/s^2)\n\n', ...
              C{c}.name, q0deg(s), grid4(1), grid4(1)*g);
      continue
    end
    if any(okv(1:top))
      fprintf('  NOTE: non-monotone -- a bound below the failing %.2f g still stabilizes\n', grid4(top));
    end
    lo = grid4(top)*g;  hi = grid4(top+1)*g;       % lo fails, hi succeeds
    it = 0;
    while (hi - lo) > 0.01*g && it < 10
      mid = 0.5*(lo + hi);  it = it + 1;
      M = simulate_fixed_ctrl(C{c}.fun, X0, P, struct('uSat', mid, 'dMs', 0, ...
                                                      'T', T, 'dwell', dwell));
      fprintf('  bisect %6.3f = %.3fg  ok=%d  %s\n', mid, mid/g, M.success, M.why);
      if M.success, hi = mid; else, lo = mid; end
    end
    fprintf('  -> %s, q0=%d deg: smallest stabilizing actuator bound in (%.3f, %.3f] m/s^2 = (%.3f, %.3f] g\n\n', ...
            C{c}.name, q0deg(s), lo, hi, lo/g, hi/g);
    R(n).thr = [lo hi];
  end
end

save(fullfile(logDir, 'exp_e4_saturation.mat'), 'R', 'grid4', 'q0deg', 'T', 'dwell');
fprintf('Saved logs/exp_e4_saturation.mat\n');
diary off
