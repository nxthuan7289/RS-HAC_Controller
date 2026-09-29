%EXP_E3_NOISE  Experiment E3: sensor noise and quantization at a FIXED control law.
%
%   Protocol:
%     - noise is added ONCE, to the measured ANGLE, ahead of the velocity estimator, so
%       that it is not counted twice. The velocity channels are not measured at all:
%       they are rebuilt by the filtered derivative of the firmware
%       (state_derivatives.cpp, T = 0.02 s), which is what the hardware does.
%     - the encoder quantum, 4000 pulses per revolution = 0.09 deg, is always applied.
%     - SNR in {40, 30, 20, 10} dB. The protocol requires the reference of the SNR to be
%       stated and NOT to be a signal that is zero at the equilibrium, so the reference
%       is the RMS of the true angle over the nominal, noise-free RS-HAC run of this
%       same scenario, printed below. sigma_q = q_ref * 10^(-SNR/20).
%     - 100 repetitions per level, and the SAME 100 noise realizations for every
%       controller (seed = 1000 + i), so the comparison is PAIRED.
%     - scenario q0 = 20 deg, nominal actuator bound, no delay, horizon 10 s.
%     - SUCCESS CRITERION, and why it differs from E4/E5. With the encoder quantum
%       differentiated by a 20 ms filter, the velocity estimate chatters by about
%       step/T = 0.0016/0.02 rad/s = 4.5 deg/s, nine times the 0.5 deg/s of the band of
%       Eq. (26). That band is a simulation band: with a measurement chain in the loop
%       NO run stays inside it, noise or no noise, so using it here would report 0 %
%       success at every SNR and measure nothing. E3 therefore separates the two questions:
%         success = the pole is kept up and the cart stays on the track for the whole
%                   horizon, judged on the TRUE state;
%         quality = RMS of the command and of the true angle over the last second.
%       RMS(u) is the noise-amplification number the protocol calls the important one.
%       The first entry into the band of Eq. (26) is still reported, as a transient
%       measure, but it is not the pass/fail criterion.
%     - failures are counted, never dropped silently; every metric below is computed
%       over the runs that survived and is labelled as such.
%
%   Two baselines are run first, because with a measurement chain in the loop the
%   noise-free case is no longer the E4/E5 case: "exact" is exact state feedback, and
%   "chain, no noise" is quantization plus the estimator with no noise. The difference
%   between them is the price of the estimator alone.
%
%   Run:  >> exp_e3_noise                Log: logs/exp_e3_noise.txt
%   (run from experiments/; the script puts ../analysis on the path itself)

clear; clc;
here   = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'analysis'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'exp_e3_noise.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('exp_e3_noise   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

q0deg = 20;
[P, C] = exp_setup(q0deg);
C = C(cellfun(@(c) ismember(c.name, {'RS-HAC', 'LQR', 'FC-clip'}), C));

X0    = [0; 0; deg2rad(q0deg); 0];
T     = 10;   dwell = 1;   nRep = 100;
stepQ = 2*pi/4000;                       % encoder quantum, 0.0015708 rad = 0.09 deg
snrDb = [40 30 20 10];
base  = struct('uSat', P.u_max, 'dMs', 0, 'T', T, 'dwell', dwell);

fprintf('q0 = %d deg, horizon %g s, dwell %g s, u_sat = nominal %.4f m/s^2, no delay\n', ...
        q0deg, T, dwell, P.u_max);
fprintf('encoder quantum %.4f deg, velocity estimator: filtered derivative, T = 20 ms\n', ...
        rad2deg(stepQ));
fprintf('%d repetitions per level, same noise realizations for every controller\n\n', nRep);

%% ---- baselines ---------------------------------------------------------------------
fprintf('=== baselines (no noise) ===\n');
fprintf('  column "alive" is the E3 criterion; column "band" is the Eq. (26) dwell criterion of E4/E5,\n');
fprintf('  kept here only to document that it is unusable once the measurement chain is in the loop.\n');
fprintf('  %-8s %-16s %5s %5s %8s %9s %9s %9s\n', 'ctrl', 'measurement', 'alive', 'band', ...
        'tEnter', 'Su', 'RMS(u)', 'RMS(q)deg');
qRef = NaN;
for c = 1:numel(C)
    for m = 1:2
        o = base;
        if m == 2
            o.meas = struct('sigma_q', 0, 'step_q', stepQ, 'estimator', true, 'seed', 1);
            tag = 'chain, no noise';
        else
            tag = 'exact state';
        end
        M = simulate_fixed_ctrl(C{c}.fun, X0, P, o);
        fprintf('  %-8s %-16s %5d %5d %8.3f %9.3f %9.4f %9.4f  %s\n', C{c}.name, tag, M.alive, ...
                M.success, M.tEnter, M.Su, M.uRms, rad2deg(M.qRms), M.why);
        if c == 1 && m == 1
            % reference for the SNR: RMS of the true angle of the nominal RS-HAC run
            qRef = rmsAngle(C{c}.fun, X0, P, base);
            fprintf('  -> SNR reference q_ref = RMS of the true angle over this run = %.4f rad = %.3f deg\n', ...
                    qRef, rad2deg(qRef));
        end
    end
end
fprintf('\n');

%% ---- noise sweep -------------------------------------------------------------------
R = struct([]);  n = 0;
for s = 1:numel(snrDb)
    sigma = qRef*10^(-snrDb(s)/20);
    fprintf('=== SNR = %d dB   sigma_q = %.5f rad = %.3f deg (%.1f encoder quanta) ===\n', ...
            snrDb(s), sigma, rad2deg(sigma), sigma/stepQ);
    fprintf('  %-8s %14s %10s %12s %12s %12s %12s\n', 'ctrl', 'success', '95% CI', ...
            'tEnter', 'Su', 'RMS(u)', 'RMS(q) deg');
    for c = 1:numel(C)
        ok = false(nRep, 1);  tE = NaN(nRep, 1);  Su = NaN(nRep, 1);
        uR = NaN(nRep, 1);    qR = NaN(nRep, 1);
        for i = 1:nRep
            o = base;
            o.meas = struct('sigma_q', sigma, 'step_q', stepQ, 'estimator', true, ...
                            'seed', 1000 + i);
            M = simulate_fixed_ctrl(C{c}.fun, X0, P, o);
            ok(i) = M.alive;
            if M.alive
                tE(i) = M.tEnter;  Su(i) = M.Su;  uR(i) = M.uRms;  qR(i) = M.qRms;
            end
        end
        [rate, lo, hi] = wilson(sum(ok), nRep);
        fprintf('  %-8s %7d/%-6d %5.0f-%3.0f%% %6.3f+-%5.3f %6.2f+-%5.2f %6.3f+-%5.3f %6.3f+-%5.3f\n', ...
                C{c}.name, sum(ok), nRep, 100*lo, 100*hi, ...
                mean(tE,'omitnan'), std(tE,'omitnan'), mean(Su,'omitnan'), std(Su,'omitnan'), ...
                mean(uR,'omitnan'), std(uR,'omitnan'), ...
                rad2deg(mean(qR,'omitnan')), rad2deg(std(qR,'omitnan')));
        n = n + 1;
        R(n).ctrl = C{c}.name;  R(n).snr = snrDb(s);  R(n).sigma = sigma;
        R(n).ok = ok;  R(n).tEnter = tE;  R(n).Su = Su;  R(n).uRms = uR;  R(n).qRms = qR;
        R(n).rate = rate;  R(n).ci = [lo hi];
    end
    fprintf('  (means and SDs are over SUCCESSFUL runs only; the success column is the full denominator)\n');

    % paired comparison on the repetitions where BOTH controllers succeeded
    iRS = find(strcmp({R.ctrl}, 'RS-HAC') & [R.snr] == snrDb(s), 1);
    for c = 2:numel(C)
        j = find(strcmp({R.ctrl}, C{c}.name) & [R.snr] == snrDb(s), 1);
        both = R(iRS).ok & R(j).ok;
        if any(both)
            d = R(iRS).uRms(both) - R(j).uRms(both);
            [mu, half] = meanCi(d);
            fprintf('  paired RMS(u), RS-HAC minus %-8s : %+0.4f +- %0.4f m/s^2 (95%% CI, n = %d pairs)\n', ...
                    C{c}.name, mu, half, sum(both));
            d = rad2deg(R(iRS).qRms(both) - R(j).qRms(both));
            [mu, half] = meanCi(d);
            fprintf('  paired RMS(q), RS-HAC minus %-8s : %+0.4f +- %0.4f deg\n', C{c}.name, mu, half);
        end
    end
    fprintf('\n');
end

%% ---- summary -----------------------------------------------------------------------
fprintf('=== lowest SNR at which the success rate is still >= 95%% ===\n');
for c = 1:numel(C)
    worst = NaN;
    for s = 1:numel(snrDb)
        j = find(strcmp({R.ctrl}, C{c}.name) & [R.snr] == snrDb(s), 1);
        if R(j).rate >= 0.95, worst = snrDb(s); end
    end
    if isnan(worst)
        fprintf('  %-8s : below 95%% already at %d dB\n', C{c}.name, snrDb(1));
    else
        fprintf('  %-8s : %d dB\n', C{c}.name, worst);
    end
end

save(fullfile(logDir, 'exp_e3_noise.mat'), 'R', 'snrDb', 'nRep', 'qRef', 'stepQ', 'q0deg');
fprintf('\nSaved logs/exp_e3_noise.mat\n');
diary off

% -----------------------------------------------------------------------------------
function r = rmsAngle(fun, X0, P, opt)
%RMSANGLE  RMS of the true angle over the whole nominal run, the SNR reference.
sub = 10;  hs = P.Ts/sub;  nK = round(opt.T/P.Ts);
X = X0(:);  q = zeros(1, nK);
for k = 1:nK
    u = min(max(fun(X), -opt.uSat), opt.uSat);
    for s = 1:sub
        k1 = cartpole_nl(X, u, P);            k2 = cartpole_nl(X + hs/2*k1, u, P);
        k3 = cartpole_nl(X + hs/2*k2, u, P);  k4 = cartpole_nl(X + hs*k3, u, P);
        X  = X + hs/6*(k1 + 2*k2 + 2*k3 + k4);
    end
    q(k) = X(3);
end
r = sqrt(mean(q.^2));
end

function [rate, lo, hi] = wilson(k, n)
%WILSON  95 % Wilson score interval for a binomial proportion.
z = 1.96;  rate = k/n;  den = 1 + z^2/n;
c  = (rate + z^2/(2*n))/den;
h  = z*sqrt(rate*(1-rate)/n + z^2/(4*n^2))/den;
lo = max(c - h, 0);  hi = min(c + h, 1);
end

function [mu, half] = meanCi(d)
%MEANCI  Mean of the paired differences and its 95 % half-width (normal approximation).
d = d(~isnan(d));  mu = mean(d);
half = 1.96*std(d)/sqrt(numel(d));
end
