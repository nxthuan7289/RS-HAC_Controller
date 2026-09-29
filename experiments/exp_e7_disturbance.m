%EXP_E7_DISTURBANCE  Experiment E7: largest recoverable impulsive disturbance.
%
%   Protocol:
%     - a torque pulse is applied to the PENDULUM at t = 3 s, width 50 ms, with the
%       amplitude increased until the run fails. The pulse shape, its instant and its
%       width are fixed; only the amplitude varies, so the comparison isolates the
%       disturbance size.
%     - the amplitude is first bracketed by doubling from 0.01 N m, then refined by
%       ten bisection steps.
%     - the run starts AT THE EQUILIBRIUM, X0 = 0, so what is measured is recovery from
%       the disturbance and not the transient of an initial angle. The pole is already
%       inside the band of Eq. (26) when the pulse arrives.
%     - the control law, the de-semantization scale of Eq. (19) and the nominal actuator
%       bound of 3 g are unchanged; no noise, no delay; horizon 10 s.
%     - success: |x| <= 0.43 m and |q| < 90 deg throughout, and the TRUE state back
%       inside the band of Eq. (26) for the last second of the horizon.
%
%   Reported per controller: the largest amplitude that is still recovered, the impulse
%   it carries, and the angular-rate step it is equivalent to, dqdot = amp*dur/(I+mL^2),
%   which is the physically meaningful number for a cart-pole.
%
%   Run:  >> exp_e7_disturbance          Log: logs/exp_e7_disturbance.txt
%   (run from experiments/; the script puts ../analysis on the path itself)

clear; clc;
here   = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'analysis'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'exp_e7_disturbance.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('exp_e7_disturbance   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

[P, C] = exp_setup(0);
C = C(cellfun(@(c) ismember(c.name, {'RS-HAC', 'LQR', 'FC-clip'}), C));

X0   = [0; 0; 0; 0];
T    = 10;  dwell = 1;
t0   = 3;   dur = 0.05;
den  = P.m*P.L^2 + P.I;
base = struct('uSat', P.u_max, 'dMs', 0, 'T', T, 'dwell', dwell);

fprintf('pulse at t = %g s, width %g ms, X0 = equilibrium, horizon %g s, dwell %g s\n', ...
        t0, 1e3*dur, T, dwell);
fprintf('u_sat = nominal %.4f m/s^2, no noise, no delay;  I + mL^2 = %.6e kg m^2\n\n', P.u_max, den);

R = struct([]);
for c = 1:numel(C)
    fprintf('=== %s ===\n', C{c}.name);
    fprintf('  %10s %5s %9s %9s %9s  %s\n', 'amp [N m]', 'ok', 'max|q|deg', 'max|x|', 'tSettle', 'note');

    run1 = @(a) simulate_fixed_ctrl(C{c}.fun, X0, P, setfield(base, 'dist', ...
                    struct('t0', t0, 'dur', dur, 'amp', a)));   %#ok<SFLD>

    % bracket: double until the run fails
    lo = 0.01;  hi = NaN;
    M  = run1(lo);
    printRow(lo, M);
    if ~M.success
        fprintf('  -> %s: fails already at the smallest amplitude tried, %.4f N m\n\n', C{c}.name, lo);
        continue
    end
    a = lo;
    for it = 1:20
        a = 2*a;
        M = run1(a);
        printRow(a, M);
        if ~M.success, hi = a; break, end
        lo = a;
    end
    if isnan(hi)
        fprintf('  -> %s: still recovers at %.4f N m, the largest amplitude tried\n\n', C{c}.name, lo);
        continue
    end

    % refine
    for it = 1:10
        mid = 0.5*(lo + hi);
        M   = run1(mid);
        fprintf('  bisect %8.4f  ok=%d  %s\n', mid, M.success, M.why);
        if M.success, lo = mid; else, hi = mid; end
    end
    dq = lo*dur/den;
    fprintf(['  -> %s: largest recovered pulse %.4f N m (fails at %.4f); impulse %.3e N m s;\n' ...
             '     equivalent angular-rate step %.1f deg/s\n\n'], ...
            C{c}.name, lo, hi, lo*dur, rad2deg(dq));
    R(c).ctrl = C{c}.name;  R(c).ampOk = lo;  R(c).ampBad = hi;
    R(c).impulse = lo*dur;  R(c).dqdot = dq;
end

fprintf('=== summary ===\n');
fprintf('  %-8s %12s %14s %16s\n', 'ctrl', 'amp [N m]', 'impulse [N m s]', 'dqdot [deg/s]');
for c = 1:numel(R)
    if isempty(R(c).ctrl), continue, end
    fprintf('  %-8s %12.4f %14.3e %16.1f\n', R(c).ctrl, R(c).ampOk, R(c).impulse, rad2deg(R(c).dqdot));
end

save(fullfile(logDir, 'exp_e7_disturbance.mat'), 'R', 't0', 'dur', 'T', 'dwell');
fprintf('\nSaved logs/exp_e7_disturbance.mat\n');
diary off

% -----------------------------------------------------------------------------------
function printRow(a, M)
fprintf('  %10.4f %5d %9.2f %9.3f %9.3f  %s\n', a, M.success, NaN, M.Dx, M.tSettle, M.why);
end
