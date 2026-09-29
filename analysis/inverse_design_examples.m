%INVERSE_DESIGN_EXAMPLES  Computing the RS-HAC parameters from a performance specification (Corollary 1).
%
%   Closed-form small-signal gain (rshac_gain.m, proof of Proposition 3), w_i = 1/4:
%       K_i  = (u_max-u_min)/4 * (th_u/th_i) * (a_u,i/a_i)^((n_i-1)/2) * ds_i
%   inverted for the resolution ratio of the intermediate controller:
%       a_u,i = a_i * [ K*_i / ((u_max-u_min)/4 * (th_u/th_i) * ds_i) ]^(2/(n_i-1))
%   The target is admissible (a_u,i in (0,1)) iff
%       0 < K*_i < K_i^max = (u_max-u_min)/4 * (th_u/th_i) * a_i^(-(n_i-1)/2) * ds_i ;
%   when K*_i >= K_i^max the label count n_i is raised by 2 (K_i^max grows by 1/a_i).
%
%   Example (i)   K* = K_LQR, Eq. (25).
%   Example (ii)  specification -> poles -> K* -> a_u:
%                 both pole pairs with damping ratio 0.8, the fast pair 4 times faster than
%                 the dominant pair; the dominant decay sigma is the smallest value on a grid
%                 for which the LINEAR closed loop reaches the band of Eq. (26) from
%                 q0 = 10 deg within Dt* = 2.0 s without exceeding u_max.
%   Both designs are checked (numerical gain, closed-loop spectrum) and simulated on the
%   nonlinear plant with the implemented controller, next to the Table 3 design and LQR.
%
%   Run:  >> inverse_design_examples          Log: logs/inverse_design_examples.txt

clear; clc;
here   = fileparts(mfilename('fullpath'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'inverse_design_examples.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('inverse_design_examples   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

P0    = rshac_params();                        % Table 3 design, extrapolating lookup (as implemented)
K_LQR = lqr_gain();                            % Eq. (25), u = +K*X
band  = [0.02; 0.02; deg2rad(0.5); deg2rad(0.5)];
X10   = [0; 0; deg2rad(10); 0];

%% ---- Example (i): K* = K_LQR ------------------------------------------------------------
fprintf('=== Example (i): target K* = K_LQR (Eq. 25) ===\n');
[Pi, repI] = inverse_design(P0, K_LQR);
report_design(Pi, repI, K_LQR);

%% ---- Example (ii): specification-driven design -------------------------------------------
fprintf('=== Example (ii): specification -> poles -> K* -> a_u ===\n');
DtStar = 2.0;  zeta = 0.8;  ratio = 4;
sigGrid = 0.50:0.05:4.00;
chosen  = NaN;
for sg = sigGrid
    wdom  = sg*tan(acos(zeta));
    poles = [-sg + 1j*wdom, -sg - 1j*wdom, ratio*(-sg + 1j*wdom), ratio*(-sg - 1j*wdom)];
    K     = -place(P0.A, P0.B, poles);
    [DtLin, uLin] = linear_band_time(P0.A + P0.B*K, K, X10, band, P0.Ts, 10);
    if DtLin <= DtStar && uLin <= P0.u_max
        chosen = sg;  Kstar = K;  polesStar = poles;  DtStarLin = DtLin;  uStarLin = uLin;
        break
    end
end
assert(~isnan(chosen), 'no dominant decay on the grid meets the specification');
assert(all(Kstar > 0), 'pole placement returned a gain with a non-positive entry');
fprintf('  specification: Dt <= %.2f s from q0 = 10 deg (linear model), zeta = %.2f, fast/dominant = %d, |u| <= u_max\n', ...
        DtStar, zeta, ratio);
fprintf('  smallest dominant decay on the grid: sigma = %.2f 1/s  -> poles %s\n', chosen, mat2str(polesStar, 4));
fprintf('  K* = %s ; linear model: Dt = %.3f s, max|u| = %.2f m/s^2\n', mat2str(Kstar, 5), DtStarLin, uStarLin);
[Pii, repII] = inverse_design(P0, Kstar);
report_design(Pii, repII, Kstar);

%% ---- nonlinear verification on the Experiment 1 scenarios ---------------------------------
fprintf('=== nonlinear plant, controller sampled at 1 ms (ZOH), saturation |u| <= u_max ===\n');
names = {'RS-HAC, Table 3', 'RS-HAC, (i) K* = K_LQR', 'RS-HAC, (ii) specification', 'LQR, Eq. (25)', 'linear K* of (ii)'};
ctrls = {@(X) rshac_law_vec(X, P0), @(X) rshac_law_vec(X, Pi), @(X) rshac_law_vec(X, Pii), ...
         @(X) K_LQR*X, @(X) Kstar*X};
fprintf('  %-28s %5s %8s %9s %9s %9s %8s\n', 'controller', 'q0', 'Dt [s]', 'Dx_m [m]', 'Su [m/s]', 'max|u|', 'success');
Sim = struct([]);
for c = 1:numel(ctrls)
    for j = 1:3
        q0 = 10*j;
        M  = simulate_exp1(ctrls{c}, [0; 0; deg2rad(q0); 0], P0, 10);
        fprintf('  %-28s %5d %8.3f %9.3f %9.3f %9.2f %8d\n', names{c}, q0, M.Dt, M.Dx, M.Su, M.uPre, M.success);
        Sim(c, j).name = names{c};  Sim(c, j).q0 = q0;  Sim(c, j).Dt = M.Dt;  Sim(c, j).Dx = M.Dx;
        Sim(c, j).Su = M.Su;  Sim(c, j).uPre = M.uPre;  Sim(c, j).success = M.success;
    end
end

save(fullfile(logDir, 'inverse_design_examples.mat'), 'Pi', 'Pii', 'repI', 'repII', 'Kstar', 'polesStar', 'Sim');
fprintf('\nSaved logs/inverse_design_examples.mat\n');
diary off;

%% ======================================================================================
function [P, rep] = inverse_design(P, Kstar)
%INVERSE_DESIGN  a_u (and, if needed, n) of every channel so that the small-signal gain is Kstar.
du  = P.u_max - P.u_min;
rep = struct('n0', zeros(1, 4), 'n', zeros(1, 4), 'Kmax', zeros(1, 4), 'au', zeros(1, 4));
for i = 1:4
    c = P.chan{i};
    rep.n0(i) = c.n;
    switch c.sem
        case 'lin', ds = 1/(c.par(2) - c.par(1));
        case 'igs', ds = c.par/4;
    end
    assert(Kstar(i) > 0, 'target gains must be positive');
    Kmax = du/4*(c.th_u/c.th_i)*c.a_i^(-(c.n - 1)/2)*ds;
    while Kstar(i) >= Kmax
        c.n  = c.n + 2;
        assert(c.n <= 21, 'no admissible label count up to 21 for channel %d', i);
        Kmax = du/4*(c.th_u/c.th_i)*c.a_i^(-(c.n - 1)/2)*ds;
    end
    c.a_u     = c.a_i*(Kstar(i)/(du/4*(c.th_u/c.th_i)*ds))^(2/(c.n - 1));
    P.chan{i} = c;
    rep.n(i) = c.n;  rep.Kmax(i) = Kmax;  rep.au(i) = c.a_u;
end
end

function report_design(P, rep, Kstar)
%REPORT_DESIGN  Print the computed parameters and check them against the implemented law.
names = {'x', 'xd', 'q', 'qd'};
fprintf('  %-4s %7s %5s %10s %10s %9s %8s\n', 'ch', 'n_old', 'n', 'K*', 'K_max', 'K*/K_max', 'a_u');
for i = 1:4
    fprintf('  %-4s %7d %5d %10.4f %10.4f %9.3f %8.4f\n', names{i}, rep.n0(i), rep.n(i), Kstar(i), ...
            rep.Kmax(i), Kstar(i)/rep.Kmax(i), rep.au(i));
end
assert(all(rep.au > 0 & rep.au < 1));
h  = 1e-6;
Kn = zeros(1, 4);
for i = 1:4
    Xp = zeros(4, 1);  Xp(i) = h;
    Kn(i) = (rshac_law(Xp, P) - rshac_law(-Xp, P))/(2*h);
end
ev = eig(P.A + P.B*Kn);
fprintf('  numerical gain of the designed RS-HAC = %s   (max rel. err %.1e)\n', mat2str(Kn, 5), max(abs(Kn - Kstar)./Kstar));
fprintf('  closed-loop spectrum = %s\n\n', mat2str(sort(ev).', 4));
assert(max(abs(Kn - Kstar)./Kstar) < 1e-6);
end

function [Dt, umax] = linear_band_time(Acl, K, X0, band, Ts, T)
%LINEAR_BAND_TIME  First time the linear closed loop from X0 satisfies Eq. (26), and max |u| until then.
Ad   = expm(Acl*Ts);
X    = X0;
Dt   = Inf;
umax = abs(K*X0);
for k = 1:round(T/Ts)
    X    = Ad*X;
    umax = max(umax, abs(K*X));
    if all(abs(X) <= band)
        Dt = k*Ts;
        return
    end
end
end
