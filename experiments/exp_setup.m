function [P, C] = exp_setup(q0deg)
%EXP_SETUP  Shared set-up and pre-check for experiments E4 and E5.
%
%   [P, C] = EXP_SETUP(q0deg) returns the nominal design P (rshac_params) and the
%   controllers of Section 4 as fixed, unsaturated control laws:
%
%     RS-HAC       rshac_law, out-of-range rule P.oor = 'extrap' (as implemented)
%     LQR          u = K X with K of Eq. (25) (lqr_gain.m), the continuous-time design of Section 4.3
%     FC (fis)     fuzzy_ctrl_v2.fis evaluated on the raw semantization
%     FC (lin)     the same rule base continued linearly over the whole semantic interval
%                  (its declared range widened from [0.25, 0.75] to [0, 1] and beyond),
%                  the counterpart of the RS-HAC 'extrap' rule (sensitivity check)
%     FC (clip)    the same rule base restricted to its declared range [0.25, 0.75];
%                  the FC of the paper
%
%   It PRINTS three pre-checks, all of which must be read before any number produced
%   by E4 or E5 is quoted:
%
%   1. the rule base is the identity out = s INSIDE its declared range, and what it
%      does outside it (it does not clamp; see the header of fc_law.m);
%   2. the small-signal gain of each law, differentiated numerically at the origin,
    %      against the gains already logged in ../analysis/logs/small_signal_margins.txt:
%        RS-HAC  [5.8669  9.4144  61.855  6.7784]   closed-form local gain
%        LQR     [14.142  11.829  56.588  7.9833]   Eq. (25)
%        FC      [34.209  7.355   58.84   3.3097]   Fig. 7 line and output scaling
%      This catches any accidental change of semantization, weights or de-semantization
%      before a sweep is run;
%   3. the semantized initial states of the scenarios, against the declared table range
%      [0.25, 0.75] shared by the SQSM tables and the FIS, so that it is on the record
%      which runs start outside the declared range of the inference tables.
%
%   No saturation inside the laws: P.saturate is forced false and the actuator bound is
%   applied only in simulate_fixed_ctrl.m.

if nargin < 1, q0deg = [10 20 30]; end

here = fileparts(mfilename('fullpath'));
P = rshac_params();
P.saturate = false;

fis   = readfis(fullfile(here, '..', 'simulation', 'fuzzy_ctrl_v2.fis'));
K_LQR = lqr_gain();                               % Eq. (25), convention u = +K*X
lo    = fis.Inputs(1).Range(1);  hi = fis.Inputs(1).Range(2);

%% ---- 1. FC rule base ---------------------------------------------------------------
ws = warning('off', 'fuzzy:general:diagEvalfis_OutOfRangeInput');
sg   = (lo:0.005:hi).';
dSwp = max(abs(evalfis(fis, [sg, 0.5*ones(numel(sg), 3)]) * [1;0;0;0] - sg));
rng(7);  Rin = lo + (hi - lo)*rand(300, 4);
dRnd = max(max(abs(evalfis(fis, Rin) - Rin)));
fprintf('FC pre-check   declared input range [%g %g]\n', lo, hi);
fprintf('  inside the range   max|evalfis(s) - s| : swept %.3e   random coupled %.3e\n', dSwp, dRnd);
oOut = evalfis(fis, [[-0.25; 0; 1; 1.25], 0.5*ones(4, 3)]);
fprintf('  outside the range  s = -0.25 / 0 / 1 / 1.25  ->  out_1 = %.4f %.4f %.4f %.4f', oOut(:, 1));
fprintf('   (undriven channels -> %.4f)\n', oOut(1, 2));
fprintf('  -> evalfis does not clamp; available conventions are selected per experiment, see fc_law.m\n');
fprintf('  (the per-sample warning %s is left OFF for the sweeps)\n\n', ws(1).identifier);

%% ---- controllers ------------------------------------------------------------------
C = {struct('name','RS-HAC',  'fun', @(X) rshac_law(X, P)), ...
     struct('name','LQR',     'fun', @(X) K_LQR*X(:)), ...
     struct('name','FC-lin',  'fun', @(X) fc_law(X, P, 'lin')), ...
     struct('name','FC-fis',  'fun', @(X) fc_law(X, P, 'fis')), ...
     struct('name','FC-clip', 'fun', @(X) fc_law(X, P, 'clip'))};

%% ---- 2. gain pre-check -------------------------------------------------------------
K_FC = [34.209 7.355 58.84 3.3097];
ref  = {[5.8669 9.4144 61.855 6.7784], K_LQR, K_FC, K_FC, K_FC};
fprintf('gain pre-check (numerical d u / d X at the origin vs ../analysis/logs/small_signal_margins.txt)\n');
for c = 1:numel(C)
    Kn = zeros(1, 4);  h = 1e-6;
    for i = 1:4
        ep = zeros(4, 1);  ep(i) = h;
        Kn(i) = (C{c}.fun(ep) - C{c}.fun(-ep))/(2*h);
    end
    fprintf('  %-8s K = %-42s  max rel. dev. %.2e\n', C{c}.name, mat2str(Kn, 5), ...
            max(abs((Kn - ref{c})./ref{c})));
end

%% ---- 3. where the scenarios start -------------------------------------------------
fprintf('\nsemantized initial states   s_i = sigma_i(x_i0), declared table range [%g %g]\n', lo, hi);
for s = 1:numel(q0deg)
    X0 = [0; 0; deg2rad(q0deg(s)); 0];
    sv = zeros(1, 4);
    for i = 1:4
        c = P.chan{i};
        switch c.sem
            case 'lin', sv(i) = (X0(i) - c.par(1))/(c.par(2) - c.par(1));
            case 'igs', sv(i) = 1/(1 + exp(-c.par*X0(i)));
        end
    end
    flag = '';
    if any(sv < lo | sv > hi), flag = '  <-- outside the declared range at t = 0'; end
    fprintf('  q0 = %2d deg   s = [%.4f %.4f %.4f %.4f]%s\n', q0deg(s), sv, flag);
end
fprintf('\n');
end
