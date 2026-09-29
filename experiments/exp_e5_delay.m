%EXP_E5_DELAY  Experiment E5: pure measurement-delay sweep at a FIXED control law.
%
%   Protocol:
%     - a pure delay is applied to the WHOLE measured state vector; the control law,
%       its scaling and the nominal actuator bound u_max = 3 g are unchanged.
%     - the grid is extended past 20 ms until failure (the small-signal predictions
%       are 28-45 ms, so a grid stopping at 20 ms could not find the threshold), then
%       bisected on the 1 ms lattice of T_s.
%     - deterministic, no noise; horizon 10 s; success = inside the band of Eq. (26)
%       for the last 1 s, |x| <= 0.43 m and |q| < 90 deg throughout.
%     - plant: Eq. (12) nominal (cartpole_nl); see the header of exp_e4_saturation.m.
%
%   The measured threshold is a LARGE-SIGNAL number and is compared, not equated, to
%   the small-signal delay margins in ../analysis/logs/small_signal_margins.txt.
%
%   Run:  >> exp_e5_delay               Log: logs/exp_e5_delay.txt
%   (run from experiments/; the script puts ../analysis on the path itself)

clear; clc;
here   = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'analysis'));          % rshac_params, rshac_law, cartpole_nl
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'exp_e5_delay.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('exp_e5_delay   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

q0deg = [10 20 30];
[P, C] = exp_setup(q0deg);
grid5 = [0 1 2 3 5 8 12 16 20 25 30 35 40 45 50 60 80 100];    % ms
T     = 10;  dwell = 1;
DM    = [44.36 43.86; 45.09 44.59; 28.50 28.00; 28.50 28.00; 28.50 28.00];  % [continuous ZOH] per controller, from ../analysis/logs/small_signal_margins.txt
fprintf('horizon %g s, dwell %g s, T_s = %g ms (delay resolution), u_sat = nominal %.4f m/s^2\n', ...
        T, dwell, 1e3*P.Ts, P.u_max);
fprintf('small-signal delay margins for reference [continuous, ZOH]:');
for c = 1:numel(C)
    fprintf('  %s %.2f/%.2f', C{c}.name, DM(c, 1), DM(c, 2));
end
fprintf(' ms   (the three FC variants share one small-signal gain)\n\n');

R = struct([]);  n = 0;
for c = 1:numel(C)
  for s = 1:numel(q0deg)
    X0 = [0; 0; deg2rad(q0deg(s)); 0];
    fprintf('=== %s,  q0 = %d deg   (small-signal DM %.2f ms, ZOH %.2f ms) ===\n', ...
            C{c}.name, q0deg(s), DM(c, 1), DM(c, 2));
    fprintf('  %7s %4s %8s %9s %8s %9s  %s\n', 'delay', 'ok', 'tEnter', 'tSettle', 'max|x|', 'Su', 'note');
    lastOK = NaN;  firstBad = NaN;
    for j = 1:numel(grid5)
      M = simulate_fixed_ctrl(C{c}.fun, X0, P, struct('uSat', P.u_max, 'dMs', grid5(j), ...
                                                      'T', T, 'dwell', dwell));
      fprintf('  %4d ms %4d %8.3f %9.3f %8.3f %9.3f  %s\n', grid5(j), M.success, ...
              M.tEnter, M.tSettle, M.Dx, M.Su, M.why);
      n = n + 1;
      R(n).ctrl = C{c}.name;  R(n).q0 = q0deg(s);  R(n).dMs = grid5(j);  R(n).M = M;
      if M.success
        lastOK = grid5(j);
      else
        firstBad = grid5(j);  break
      end
    end
    if isnan(firstBad)
      fprintf('  -> %s, q0=%d deg: still stabilizes at the largest delay tried, %d ms\n\n', ...
              C{c}.name, q0deg(s), grid5(end));
      continue
    end
    if isnan(lastOK)
      fprintf('  -> %s, q0=%d deg: FAILS already at zero delay\n\n', C{c}.name, q0deg(s));
      continue
    end
    lo = lastOK;  hi = firstBad;                    % lo succeeds, hi fails
    while hi - lo > 1
      mid = floor(0.5*(lo + hi));
      M = simulate_fixed_ctrl(C{c}.fun, X0, P, struct('uSat', P.u_max, 'dMs', mid, ...
                                                      'T', T, 'dwell', dwell));
      fprintf('  bisect %4d ms  ok=%d  %s\n', mid, M.success, M.why);
      if M.success, lo = mid; else, hi = mid; end
    end
    fprintf('  -> %s, q0=%d deg: critical delay between %d and %d ms\n\n', C{c}.name, q0deg(s), lo, hi);
    R(n).thr = [lo hi];
  end
end

save(fullfile(logDir, 'exp_e5_delay.mat'), 'R', 'grid5', 'q0deg', 'T', 'dwell');
fprintf('Saved logs/exp_e5_delay.mat\n');
diary off
