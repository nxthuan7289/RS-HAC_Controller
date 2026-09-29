%VALIDATE_INVARIANT_SET  Numerical check of the quadratic certificate.
%
%   Loads certificate_lmi.mat (rshac_lmi_certificate.m) and simulates the NONLINEAR
%   cart-pole, Eq. (12), under the implemented RS-HAC (explicit saturation on) from
%   initial states on the boundary of each certified ellipsoid E = {X' inv(Q) X <= 1}.
%   The certificate is for the stated plant and continuous-time control law.
%   Sampled simulations are reported separately and are not covered by it.
%
%   Two implementations of the controller are simulated:
%     continuous  u = u(X(t)) at every integration stage -- what the certificate covers
%     sampled     u held for T_s = 1 ms, as in the firmware -- reported, not asserted
%
%   Exports the (q, qd) and (x, xd) projections of the ellipsoid and certified box.
%
%   Run:  >> validate_invariant_set          Log: logs/validate_invariant_set.txt
%   Out:  certificate_projections.mat, logs/certified_ellipse_q_qd_deg.csv

clear; clc;
here   = fileparts(mfilename('fullpath'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'validate_invariant_set.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('validate_invariant_set   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

P = rshac_params();
P.saturate = true;
C = load(fullfile(here, 'certificate_lmi.mat'));

%% ---- the vectorised law equals rshac_law ---------------------------------------------
rng(3);
env = [P.env.x; P.env.xd; P.env.q; P.env.qd];
Xt  = (2*rand(4, 500) - 1).*env;
Xt(3, 1:50) = P.l1;                                   % weight boundaries
Xt(3, 51:60) = -P.l2;
uv  = rshac_law_vec(Xt, P);
err = 0;
for j = 1:size(Xt, 2)
    err = max(err, abs(rshac_law(Xt(:, j), P) - uv(j)));
end
fprintf('rshac_law_vec vs rshac_law on %d states (incl. |q| = l1, l2): max |du| = %.2e\n\n', size(Xt, 2), err);
assert(err < 1e-12);

%% ---- simulations from the boundary of each certified ellipsoid -------------------------
N  = 1000;
T  = 5;
h  = 5e-4;
f  = @(X) cartpole_nl(X, rshac_law_vec(X, P), P);
th = linspace(0, 2*pi, 361);
Proj = struct([]);
for mdl = 1:numel(C.S)
    S   = C.S(mdl);
    Q   = S.Q;
    Qi  = inv(Q);
    lam = S.lam0;
    fprintf('=== certificate for the %s plant model: rho* = %.4f, lam0 = %g ===\n', S.plant, S.rho, lam);

    Z  = randn(4, N);
    X0 = chol(Q, 'lower')*(Z./vecnorm(Z));            % X0' inv(Q) X0 = 1
    assert(max(abs(sum(X0.*(Qi*X0), 1) - 1)) < 1e-10);

    % continuous-time control, RK4
    X = X0;  maxV = ones(1, N);  maxRatio = ones(1, N);  outBox = false(1, N);  uPeak = 0;
    for k = 1:round(T/h)
        k1 = f(X);  k2 = f(X + h/2*k1);  k3 = f(X + h/2*k2);  k4 = f(X + h*k3);
        X  = X + h/6*(k1 + 2*k2 + 2*k3 + k4);
        V  = sum(X.*(Qi*X), 1);
        maxV     = max(maxV, V);
        maxRatio = max(maxRatio, V/exp(-2*lam*k*h));
        outBox   = outBox | any(abs(X) > S.Xbar(:)*(1 + 1e-9), 1);
        uPeak    = max(uPeak, max(abs(rshac_law_vec(X, rmfield(P, 'saturate')))));
    end
    VT = sum(X.*(Qi*X), 1);
    fprintf('  continuous control, %d trajectories, T = %g s:\n', N, T);
    fprintf('    max_t V(t) = %.6f (<= 1 required),  max_t V(t)exp(2 lam0 t) = %.6f (<= 1 required)\n', max(maxV), max(maxRatio));
    fprintf('    trajectories leaving the certified box: %d;  peak unsaturated |u| = %.2f m/s^2 (u_max = %.2f)\n', nnz(outBox), uPeak, P.u_max);
    fprintf('    V(T): max %.3e, median %.3e  (bound exp(-2 lam0 T) = %.3e)\n', max(VT), median(VT), exp(-2*lam*T));
    assert(max(maxV) <= 1 + 1e-6 && max(maxRatio) <= 1 + 1e-6 && ~any(outBox), 'certificate violated in simulation');

    % sampled-data control (ZOH, T_s = 1 ms), RK4 with 10 sub-steps per sample
    X = X0;  maxVs = ones(1, N);  hs = P.Ts/10;
    for kc = 1:round(T/P.Ts)
        u = rshac_law_vec(X, P);
        for j = 1:10
            k1 = cartpole_nl(X, u, P);             k2 = cartpole_nl(X + hs/2*k1, u, P);
            k3 = cartpole_nl(X + hs/2*k2, u, P);   k4 = cartpole_nl(X + hs*k3, u, P);
            X  = X + hs/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        maxVs = max(maxVs, sum(X.*(Qi*X), 1));
    end
    VTs = sum(X.*(Qi*X), 1);
    fprintf('  sampled control (ZOH 1 ms): max_t V(t) = %.6f;  V(T): max %.3e;  converged (V(T) < 1e-3): %d of %d\n\n', ...
            max(maxVs), max(VTs), nnz(VTs < 1e-3), N);

    Proj(mdl).plant   = S.plant;
    Proj(mdl).q_qd    = sqrtm(Q([3 4], [3 4]))*[cos(th); sin(th)];     % boundary of the projection of E
    Proj(mdl).x_xd    = sqrtm(Q([1 2], [1 2]))*[cos(th); sin(th)];
    Proj(mdl).box     = S.Xbar;
    Proj(mdl).maxV    = max(maxV);
    Proj(mdl).maxVzoh = max(maxVs);
end

save(fullfile(here, 'certificate_projections.mat'), 'Proj');
nl = find(strcmp({Proj.plant}, 'nonlinear'), 1);
writematrix([rad2deg(Proj(nl).q_qd(1, :)).', rad2deg(Proj(nl).q_qd(2, :)).'], fullfile(logDir, 'certified_ellipse_q_qd_deg.csv'));
fprintf('Saved certificate_projections.mat and logs/certified_ellipse_q_qd_deg.csv (q [deg], qd [deg/s])\n');
diary off;
