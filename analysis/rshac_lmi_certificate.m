%RSHAC_LMI_CERTIFICATE  Level-2 quadratic stability certificate solved with LMI Lab.
%
%   SUPPLEMENTARY: this certificate is not reported in the paper, whose stability
%   result is the local one of Proposition 3. It replaces the projected-subgradient
%   search of rshac_stability.m, whose box (certificate.mat, |q| <= 1.17 deg with
%   |x| <= 1.29 m, logs/T0_certificate_2026-09-07.txt) depends on the solver.
%
%   Closed loop on the box |X_i| <= Xbar_i, controller unchanged (Lur'e form):
%       dX/dt = [A(sigma) + B(c) (w .* k)] X
%       k_i   in [k_i^min, k_i^max]    sector of the unweighted action u_i(x_i)/x_i on the box
%       w     in conv{W}               weights of Eqs. (20)-(22) for |q| <= qbar
%       sigma in [sin(qbar)/qbar, 1]   gravity term of Eq. (12)   (nonlinear plant only)
%       c     in [cos(qbar), 1]        input term of Eq. (12)     (nonlinear plant only)
%   The map (sigma, c, w, k) -> closed-loop matrix is affine in each factor, so the
%   LMI is imposed at every vertex (at most 2*2*3*16 = 192).
%
%   Certificate: Q > 0 with  M_v Q + Q M_v' + 2*lam*Q < 0  at every vertex v.
%   V(X) = X' inv(Q) X then decays at rate >= 2*lam along every trajectory that stays
%   in the box, so the ellipsoid {X' inv(Q) X <= 1}, placed inside the box, is
%   invariant and exponentially attracting.
%
%   Box: one scale rho applied to the balance envelope P.env, found by bisection.
%   This makes the certified region unique and reproducible (greedy coordinate
%   ascent is not: it returns different, non-comparable boxes).
%
%   Run:  >> rshac_lmi_certificate          Log: logs/rshac_lmi_certificate.txt
%   Out:  certificate_lmi.mat

clear; clc;
here   = fileparts(mfilename('fullpath'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'rshac_lmi_certificate.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('rshac_lmi_certificate   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

P     = rshac_params();
shape = [P.env.x P.env.xd P.env.q P.env.qd];
lam0  = 0.01;                                  % dV/dt <= -2*lam0*V
rhoRef = 0.1620/P.env.x;                    % reference box |x| <= 0.162 m (uniform scale 0.3767)
fprintf('envelope: |x| <= %.2f m, |xd| <= %.1f m/s, |q| <= %.0f deg, |qd| <= %.0f deg/s;  oor = %s;  lam0 = %g 1/s\n\n', ...
        shape(1), shape(2), rad2deg(shape(3)), rad2deg(shape(4)), P.oor, lam0);

%% ---- consistency of the vectorised channel evaluation with rshac_law ------------------
rng(2);
for t = 1:200
    X = (2*rand(4, 1) - 1).*shape(:);
    [~, ui] = rshac_law(X, P);
    for i = 1:4
        assert(abs(channel_u(i, X(i), P) - ui(i)) < 1e-12, 'channel_u disagrees with rshac_law');
    end
end

%% ---- bisection on rho for both plant models ------------------------------------------
models = {'linearised', 'nonlinear'};
S = struct([]);
for mdl = 1:numel(models)
    plant = models{mdl};
    fprintf('=== plant model: %s ===\n', plant);

    [Mv, info] = vertices(shape, P, plant);
    [ok1, mu1] = lmi_feasible(Mv, lam0);
    fprintf('  full envelope (rho = 1): %d vertices, %d not Hurwitz, best decay mu* = %.4g -> certified: %d\n', ...
            numel(Mv), info.nNotHurwitz, mu1, ok1);

    [Mv, info] = vertices(rhoRef*shape, P, plant);
    [okL, muL] = lmi_feasible(Mv, lam0);
    fprintf('  reference box (rho = %.4f): %d vertices, mu* = %.4g -> certified: %d\n', ...
            rhoRef, numel(Mv), muL, okL);

    if ok1
        rho = 1;
    else
        lo = 0;  hi = 1;
        while hi - lo > 1e-4
            mid = (lo + hi)/2;
            if lmi_feasible(vertices(mid*shape, P, plant), lam0), lo = mid; else, hi = mid; end
        end
        rho = lo;
    end
    Xbar = rho*shape;
    [Mv, info] = vertices(Xbar, P, plant);
    [okR, muR, Qd] = lmi_feasible(Mv, lam0);
    assert(okR, 'bisection result not certified');

    [Q, gam] = lmi_max_ellipsoid(Mv, lam0, Xbar);
    worst = verify_Q(Mv, Q, lam0);
    assert(all(eig(Q) > 0) && worst < 0 && all(diag(Q).' <= Xbar.^2*(1 + 1e-6)), 'ellipsoid certificate fails verification');
    ext = sqrt(diag(Q)).';

    fprintf('  CERTIFIED BOX  rho* = %.4f:  |x| <= %.4f m, |xd| <= %.4f m/s, |q| <= %.3f deg, |qd| <= %.2f deg/s\n', ...
            rho, Xbar(1), Xbar(2), rad2deg(Xbar(3)), rad2deg(Xbar(4)));
    fprintf('    %d vertices; sectors k_min = %s, k_max = %s\n', numel(Mv), mat2str(info.k1, 4), mat2str(info.k2, 4));
    fprintf('    best decay on this box: mu* = %.4g (lam* = %.4g 1/s);  |u| <= %.2f m/s^2 on the box (u_max = %.2f)\n', ...
            muR, -muR/2, info.uBound, P.u_max);
    fprintf('    invariant ellipsoid {X''inv(Q)X <= 1}: gamma* = %.4f; extents |x| %.4f m, |xd| %.4f m/s, |q| %.3f deg, |qd| %.2f deg/s\n', ...
            gam, ext(1), ext(2), rad2deg(ext(3)), rad2deg(ext(4)));
    fprintf('    independent check: max_v lambda_max(M_v Q + Q M_v'' + 2 lam0 Q)/lambda_max(Q) = %.3e < 0\n', worst);
    disp('    Q ='); disp(Q);
    fprintf('\n');

    S(mdl).plant = plant;   S(mdl).rho = rho;     S(mdl).Xbar = Xbar;  S(mdl).lam0 = lam0;
    S(mdl).k1 = info.k1;    S(mdl).k2 = info.k2;  S(mdl).W = info.W;   S(mdl).nVertices = numel(Mv);
    S(mdl).muStar = muR;    S(mdl).Qdecay = Qd;   S(mdl).Q = Q;        S(mdl).gamma = gam;
    S(mdl).extent = ext;    S(mdl).uBound = info.uBound;
    S(mdl).fullEnvelopeCertified = ok1;  S(mdl).refBoxCertified = okL;
end

%% ---- sensitivity of rho* to the required decay rate (nonlinear plant) ----------------
fprintf('=== sensitivity of rho* to lam0 (nonlinear plant) ===\n');
lamList = [0.001 0.01 0.1 0.3 0.5];
rhoList = zeros(size(lamList));
for m = 1:numel(lamList)
    lo = 0;  hi = 1;
    if lmi_feasible(vertices(shape, P, 'nonlinear'), lamList(m)), lo = 1; end
    while hi - lo > 1e-3
        mid = (lo + hi)/2;
        if lmi_feasible(vertices(mid*shape, P, 'nonlinear'), lamList(m)), lo = mid; else, hi = mid; end
    end
    rhoList(m) = lo;
    fprintf('  lam0 = %-6g rho* = %.4f  (|q| <= %.2f deg)\n', lamList(m), lo, rad2deg(lo*shape(3)));
end

save(fullfile(here, 'certificate_lmi.mat'), 'S', 'shape', 'lamList', 'rhoList');
fprintf('\nSaved certificate_lmi.mat\n');
diary off;

%% ======================================================================================
function u = channel_u(i, v, P)
%CHANNEL_U  Unweighted intermediate action u_i(v) of rshac_law (stages 1-3), vectorised in v.
c = P.chan{i};
switch c.sem
    case 'lin', s = (v - c.par(1))/(c.par(2) - c.par(1));
    case 'igs', s = 1./(1 + exp(-c.par*v));
end
bp = sqsm_values(c.n, c.a_i, c.th_i);
tb = sqsm_values(c.n, c.a_u, c.th_u);
switch P.oor
    case 'extrap', us = interp1(bp, tb, s, 'linear', 'extrap');
    case 'clip',   us = interp1(bp, tb, min(max(s, bp(1)), bp(end)), 'linear');
    case 'haend',  us = interp1([0 bp 1], [0 tb 1], min(max(s, 0), 1), 'linear');
end
u = us*(P.u_max - P.u_min) + P.u_min;
end

function [kmin, kmax] = sector(i, Vbar, P)
%SECTOR  Extremes of u_i(v)/v on 0 < |v| <= Vbar. u_i is odd (symmetry of Proposition 1), so v > 0 suffices.
%   Dense grid plus the lookup breakpoints and the small-signal limit, refined by
%   fminbnd around the extreme grid points.
c  = P.chan{i};
bp = sqsm_values(c.n, c.a_i, c.th_i);
switch c.sem
    case 'lin', vb = bp*(c.par(2) - c.par(1)) + c.par(1);
    case 'igs', vb = log(bp./(1 - bp))/c.par;
end
vb = vb(vb > 0 & vb < Vbar);
v  = unique([linspace(Vbar*1e-6, Vbar, 20001), vb, Vbar]);
r  = channel_u(i, v, P)./v;
K0 = rshac_gain(P);
k0 = 4*K0(i);                                  % limit of u_i/v as v -> 0 (unweighted)
f  = @(x) channel_u(i, x, P)./x;
[rmin, jmin] = min(r);
[rmax, jmax] = max(r);
[~, fm] = fminbnd(f, v(max(jmin - 1, 1)), v(min(jmin + 1, numel(v))));
[~, fM] = fminbnd(@(x) -f(x), v(max(jmax - 1, 1)), v(min(jmax + 1, numel(v))));
kmin = min([rmin, fm, k0]);
kmax = max([rmax, -fM, k0]);
end

function W = weight_vertices(qbar, P)
%WEIGHT_VERTICES  Generators of the convex hull of w(|q|) for |q| <= qbar, Eqs. (20)-(22).
W = [0.25 0.25 0.25 0.25];
if qbar > P.l1
    for q = [P.l1*(1 + 1e-12), min(qbar, P.l2)]
        [~, ~, w] = rshac_law([0; 0; q; 0], P);
        W = [W; w]; %#ok<AGROW>
    end
    if qbar > P.l2, W = [W; 0 0 1 0]; end
end
end

function [Mv, info] = vertices(Xbar, P, plant)
%VERTICES  Closed-loop vertex matrices on the box |X_i| <= Xbar_i.
den = P.m*P.L^2 + P.I;
k1  = zeros(1, 4);  k2 = zeros(1, 4);
for i = 1:4, [k1(i), k2(i)] = sector(i, Xbar(i), P); end
W = weight_vertices(Xbar(3), P);
switch plant
    case 'linearised', sig = 1;                            cq = 1;
    case 'nonlinear',  sig = [sin(Xbar(3))/Xbar(3), 1];   cq = [cos(Xbar(3)), 1];
end
Mv = {};
for a = sig
    for b = cq
        A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 P.m*P.g*P.L/den*a, -P.k/den];
        B = [0; 1; 0; -P.m*P.L/den*b];
        for r = 1:size(W, 1)
            for m = 0:15
                bits = bitget(m, 1:4);
                k    = k1.*(1 - bits) + k2.*bits;
                Mv{end+1} = A + B*(W(r, :).*k); %#ok<AGROW>
            end
        end
    end
end
info.k1 = k1;  info.k2 = k2;  info.W = W;
info.nNotHurwitz = sum(cellfun(@(M) any(real(eig(M)) >= 0), Mv));
info.uBound = max(W*(k2.*Xbar).');             % |u| <= sum_i w_i k_i^max Xbar_i
end

function [ok, mu, Q] = lmi_feasible(Mv, lam)
%LMI_FEASIBLE  Best common decay: minimise mu s.t. M_v Q + Q M_v' < mu Q, Q > I (gevp).
%   Certified for lam when mu < -2*lam and the returned Q passes an eigenvalue check.
Q  = [];
nv = numel(Mv);
if any(cellfun(@(M) max(real(eig(M))) >= -lam, Mv))    % necessary: every vertex decays faster than lam
    ok = false;  mu = max(cellfun(@(M) 2*max(real(eig(M))), Mv));
    return
end
setlmis([]);
Qv = lmivar(1, [4 1]);
lmiterm([1 1 1 0], 1);                         % I < Q
lmiterm([-1 1 1 Qv], 1, 1);
lmiterm([-2 1 1 Qv], 1, 1);                    % 0 < Q
for v = 1:nv
    lmiterm([v+2 1 1 Qv], Mv{v}, 1, 's');      % M Q + Q M'
    lmiterm([-(v+2) 1 1 Qv], 1, 1);            %   < mu Q
end
lmis = getlmis;
[mu, xopt] = gevp(lmis, nv, [1e-4 500 1e9 50 1]);
if isempty(mu)
    ok = false;  mu = Inf;
    return
end
Q  = dec2mat(lmis, xopt, Qv);
ok = mu < -2*lam && verify_Q(Mv, Q, lam) < 0;
end

function [Q, gam] = lmi_max_ellipsoid(Mv, lam, Xbar)
%LMI_MAX_ELLIPSOID  max gamma s.t. decay LMIs, gamma*diag(Xbar.^2) <= Q, Q_ii <= Xbar_i^2 (mincx).
nv = numel(Mv);
setlmis([]);
Qv = lmivar(1, [4 1]);
Gv = lmivar(1, [4 0]);                         % gamma * I
for v = 1:nv
    lmiterm([v 1 1 Qv], Mv{v}, 1, 's');        % M Q + Q M'
    lmiterm([v 1 1 Qv], lam, 1, 's');          % + 2 lam Q
end
k  = nv + 1;
Dh = diag(Xbar);
lmiterm([k 1 1 Gv], Dh, Dh);                   % gamma*D < Q
lmiterm([-k 1 1 Qv], 1, 1);
for i = 1:4
    e = zeros(1, 4);  e(i) = 1;
    k = k + 1;
    lmiterm([k 1 1 Qv], e, e.');               % Q_ii < Xbar_i^2
    lmiterm([-k 1 1 0], Xbar(i)^2);
end
k = k + 1;
lmiterm([-k 1 1 Gv], 1, 1);                    % gamma > 0
lmis = getlmis;
nd   = decnbr(lmis);
cvec = zeros(nd, 1);
for j = 1:nd
    [~, Gj] = defcx(lmis, j, Qv, Gv);
    cvec(j) = -Gj(1, 1);
end
[copt, xopt] = mincx(lmis, cvec, [1e-5 500 0 50 1]);
Q   = dec2mat(lmis, xopt, Qv);
gam = -copt;
end

function worst = verify_Q(Mv, Q, lam)
%VERIFY_Q  max over vertices of lambda_max(M Q + Q M' + 2 lam Q), scaled by lambda_max(Q).
worst = -Inf;
for v = 1:numel(Mv)
    S = Mv{v}*Q + Q*Mv{v}.' + 2*lam*Q;
    worst = max(worst, max(eig((S + S.')/2)));
end
worst = worst/max(eig(Q));
if min(eig(Q)) <= 0, worst = Inf; end
end
