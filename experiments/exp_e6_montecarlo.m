%EXP_E6_MONTECARLO  Experiment E6: Monte-Carlo parameter uncertainty sweep.
%
%   Protocol:
%     - Parameters m, L, k are drawn uniformly in +/-30% around their nominal values.
%       I is calculated as I = m*L^2/3 to keep physical consistency.
%     - The controller and its gains are NOT redesigned for each plant instance;
%       LQR keeps its nominal gain.
%     - 500 sets of parameters are drawn and used across all controllers.
%     - Scenario: q0 = 20 deg, no noise.
%     - Output: success rate, 95% Wilson confidence interval, mean +/- SD of metrics.
%
%   Run:  >> exp_e6_montecarlo          Log: logs/exp_e6_montecarlo.txt
%   (run from experiments/; the script puts ../analysis on the path itself)

clear; clc;
here   = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'analysis'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'exp_e6_montecarlo.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('exp_e6_montecarlo   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

q0deg = [20];
[P_nom, C] = exp_setup(q0deg);
C = C(cellfun(@(c) ismember(c.name, {'RS-HAC', 'LQR', 'FC-clip'}), C));

N = 500;
spread = 0.30;
fprintf('Running %d Monte-Carlo iterations with +/- %g%% uniform parameter uncertainty.\n', N, spread*100);
fprintf('Nominal m = %g, L = %g, k = %g\n\n', P_nom.m, P_nom.L, P_nom.k);

rng(20260923); % fixed seed for reproducibility
Pm = P_nom.m * (1 + spread * (2*rand(N, 1) - 1));
PL = P_nom.L * (1 + spread * (2*rand(N, 1) - 1));
Pk = P_nom.k * (1 + spread * (2*rand(N, 1) - 1));

R = struct([]);
T = 10; dwell = 1;
X0 = [0; 0; deg2rad(q0deg); 0];

for c = 1:numel(C)
    fprintf('=== %s ===\n', C{c}.name);
    successes = false(N, 1);
    Su_list = NaN(N, 1);
    Dx_list = NaN(N, 1);
    tEnter_list = NaN(N, 1);
    tSettle_list = NaN(N, 1);
    
    tic;
    for i = 1:N
        P = P_nom;
        P.m = Pm(i);
        P.L = PL(i);
        P.k = Pk(i);
        P.I = (1/3)*P.m*P.L^2;
        % Plant A and B matrices are only used for LQR design, not for cartpole_nl.
        % So no need to update P.A and P.B here since control law is fixed.
        
        M = simulate_fixed_ctrl(C{c}.fun, X0, P, struct('uSat', P_nom.u_max, 'dMs', 0, 'T', T, 'dwell', dwell));
        
        successes(i) = M.success;
        if M.success
            Su_list(i) = M.Su;
            Dx_list(i) = M.Dx;
            tEnter_list(i) = M.tEnter;
            tSettle_list(i) = M.tSettle;
        end
    end
    t_elapsed = toc;
    
    % Statistics
    n_succ = sum(successes);
    rate = n_succ / N;
    
    % Wilson Score Interval (95% CI, z ~= 1.96)
    z = 1.96;
    denom = 1 + z^2/N;
    center = (rate + z^2/(2*N)) / denom;
    half_width = z * sqrt(rate*(1-rate)/N + z^2/(4*N^2)) / denom;
    ci_lower = center - half_width;
    ci_upper = center + half_width;
    
    fprintf('Success rate: %.1f%% (%d/%d), 95%% CI: [%.1f%%, %.1f%%]\n', ...
        rate*100, n_succ, N, ci_lower*100, ci_upper*100);
    
    if n_succ > 0
        fprintf('  (the four lines below are over SUCCESSFUL runs only, n = %d; the failures are in the rate above)\n', n_succ);
        fprintf('  Su (control effort) : %7.3f +/- %6.3f\n', mean(Su_list, 'omitnan'), std(Su_list, 'omitnan'));
        fprintf('  Dx (max cart pos)   : %7.3f +/- %6.3f m\n', mean(Dx_list, 'omitnan'), std(Dx_list, 'omitnan'));
        fprintf('  tEnter              : %7.3f +/- %6.3f s\n', mean(tEnter_list, 'omitnan'), std(tEnter_list, 'omitnan'));
        fprintf('  tSettle             : %7.3f +/- %6.3f s\n', mean(tSettle_list, 'omitnan'), std(tSettle_list, 'omitnan'));
    end
    fprintf('  Time elapsed: %.1f s\n\n', t_elapsed);
    
    R(c).ctrl = C{c}.name;
    R(c).successes = successes;
    R(c).Su_list = Su_list;
    R(c).Dx_list = Dx_list;
    R(c).tEnter_list = tEnter_list;
    R(c).tSettle_list = tSettle_list;
    R(c).rate = rate;
    R(c).ci = [ci_lower, ci_upper];
end

%% ---- paired analysis: the SAME 500 plants were used for every controller ----------
fprintf('=== paired comparison on the same plant draws (RS-HAC minus baseline) ===\n');
iRS = find(strcmp({R.ctrl}, 'RS-HAC'), 1);
for c = 1:numel(R)
    if c == iRS, continue, end
    both   = R(iRS).successes & R(c).successes;
    nb     = sum(both);
    onlyRS = sum(R(iRS).successes & ~R(c).successes);
    onlyOt = sum(~R(iRS).successes & R(c).successes);
    fprintf('  vs %-8s : both stabilize %d, only RS-HAC %d, only %s %d (discordant %d)\n', ...
            R(c).ctrl, nb, onlyRS, R(c).ctrl, onlyOt, onlyRS + onlyOt);
    if nb > 1
        d  = R(iRS).Su_list(both) - R(c).Su_list(both);
        fprintf('    paired Su      : %+0.3f +- %0.3f   (95%% CI of the mean difference, n = %d)\n', ...
                mean(d), 1.96*std(d)/sqrt(nb), nb);
        d  = R(iRS).tSettle_list(both) - R(c).tSettle_list(both);
        fprintf('    paired tSettle : %+0.3f +- %0.3f s\n', mean(d), 1.96*std(d)/sqrt(nb));
    end
end
fprintf('\n');

save(fullfile(logDir, 'exp_e6_montecarlo.mat'), 'R', 'N', 'Pm', 'PL', 'Pk', 'P_nom', 'q0deg');
fprintf('Saved logs/exp_e6_montecarlo.mat\n');
diary off

