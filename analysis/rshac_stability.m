%RSHAC_STABILITY  Stability calculations for the nominal RS-HAC balance model.
%
%   This script analyzes the control law implemented in rshac_law.m and
%   my_HAC_improved.m. It does not modify the controller.
%
%   Level 1  local exponential stability from the closed-loop Jacobian.
%   Level 2  Lur'e / polytopic quadratic certificate on an explicit box, with
%            the box maximised by coordinate ascent. SUPPLEMENTARY: not reported
%            in the paper, whose stability result is Level 1 (Proposition 3).
%   Level 3  simulated trajectory checks are reported separately.
%
%   MODEL STRUCTURE
%   -----------------------------------------------------
%   RS-HAC = linear plant (Eq. 13) + four DECOUPLED monotone static
%   nonlinearities u_i(x_i) (Eq. 11 breakpoints, piecewise-linear interpolation)
%   + a weighted sum whose weights live in a finite-vertex polytope (Eqs. 20-22).
%   That is exactly a Lur'e system, so absolute-stability tools apply as-is.
%
%   SCOPE
%   ----------------------------
%   On the FULL balance envelope the quadratic certificate does NOT exist: some
%   vertices are not even Hurwitz (low q-dot sector gain from UGS saturation, and
%   low q gain combined with high x gain). The script therefore reports the
%   LARGEST box on which the certificate does hold. This is not a global
%   stability result.
%
%   Run:  >> rshac_stability

clear; clc; rng(0);
P = rshac_params();
keys = {'x','xd','q','qd'};
base = [P.env.x P.env.xd P.env.q P.env.qd];

%% ================= Level 1: Proposition 3 ===========================
K_RS = smallSignalGain(P);
ev   = eig(P.A + P.B*K_RS);
fprintf('=== Level 1: local exponential stability ===\n');
fprintf('  K_RS  = %s\n', mat2str(K_RS,6));
fprintf('  eig   = %s\n', mat2str(sort(ev).',5));
fprintf('  Hurwitz = %d\n\n', all(real(ev) < 0));
assert(all(real(ev) < 0), 'Proposition 3 fails');

%% ======================= Level 2: certified box =========================
fprintf('=== Level 2: polytopic quadratic certificate ===\n');

% (a) full envelope -- expected to FAIL; we report this explicitly
[okFull, wFull, ~, nbadFull, nvFull] = certify(ones(1,4), base, P);
fprintf('  full envelope        : feasible=%d  worst=%+.4e  (%d/%d vertices not Hurwitz)\n', ...
        okFull, wFull, nbadFull, nvFull);

% (b) uniform bisection
lo = 0.02; hi = 1.0;
for i = 1:14
    mid = (lo+hi)/2;
    if certify(mid*ones(1,4), base, P), lo = mid; else, hi = mid; end
end
fprintf('  uniform bisection    : rho* = %.4f\n', lo);

% (c) coordinate ascent to enlarge the box channel by channel
sc = lo*ones(1,4);
for sweep = 1:3
    for j = 1:4
        a = sc(j); b = 3.0;
        for i = 1:11
            mid = (a+b)/2; t = sc; t(j) = mid;
            if certify(t, base, P), a = mid; else, b = mid; end
        end
        sc(j) = a;
    end
    fprintf('  coord. ascent sweep %d: %s\n', sweep, mat2str(sc,4));
end

[ok, worst, Pm, ~, nv] = certify(sc, base, P, 6);
lim = sc .* base;
fprintf('\n  --- certified box (supplementary, not reported in the paper) ---\n');
fprintf('   feasible = %d,  worst vertex eig(Av''P+PAv) = %.4e,  %d vertices\n', ok, worst, nv);
fprintf('   |x|   <= %.4f m\n',      lim(1));
fprintf('   |xd|  <= %.4f m/s\n',    lim(2));
fprintf('   |q|   <= %.2f deg\n',    rad2deg(lim(3)));
fprintf('   |qd|  <= %.1f deg/s\n',  rad2deg(lim(4)));
disp('   P ='); disp(Pm);
assert(ok, 'no quadratic certificate found -- fall back to Level 1 + Level 3 only');

% largest invariant ellipsoid {X : X''PX <= c} inscribed in the certified box
Pi = inv(Pm);
c  = min( (lim.^2) ./ diag(Pi).' );
ax = sqrt(c*diag(Pi)).';
fprintf('\n   invariant ellipsoid  c* = %.6g\n', c);
fprintf('   semi-axes: %.4f m, %.4f m/s, %.2f deg, %.1f deg/s\n', ...
        ax(1), ax(2), rad2deg(ax(3)), rad2deg(ax(4)));

save(fullfile(fileparts(mfilename('fullpath')),'certificate.mat'), ...
     'Pm','sc','lim','c','ax','K_RS');
fprintf('\nSaved certificate.mat\n');

%% ======================================================================
function K = smallSignalGain(P)
%SMALLSIGNALGAIN  Closed-form K_RS (see verify_gain_formula.m for the check).
K = zeros(1,numel(P.chan));
for i = 1:numel(P.chan)
    c  = P.chan{i};
    g0 = (c.a_u/c.a_i)^((c.n-1)/2) * (c.th_u/c.th_i);
    switch c.sem
        case 'lin', ds = 1/(c.par(2)-c.par(1));
        case 'igs', ds = c.par/4;
    end
    K(i) = 0.25*(P.u_max-P.u_min)*g0*ds;
end
end

function [k1,k2] = sectorBound(i, lim, P)
%SECTORBOUND  Exact sector [k_min,k_max] of u_i(v)/v for |v| <= lim.
v = linspace(-lim, lim, 4001); v(abs(v) < 1e-12) = [];
r = zeros(size(v));
for j = 1:numel(v)
    X = zeros(4,1); X(i) = v(j);
    [~, ui] = rshac_law(X, P);
    r(j) = ui(i)/v(j);
end
k1 = min(r); k2 = max(r);
end

function W = weightVertices(qlim, P)
%WEIGHTVERTICES  Convex hull generators of w(|q|) for |q| <= qlim (Eqs. 20-22).
W = [0.25 0.25 0.25 0.25];
if qlim > P.l1
    for q = [P.l1+1e-9, qlim]
        wq  = min(0.25 + (q-P.l1)*0.75/(P.l2-P.l1), 1);
        wqd = (1-wq)/2; wx = (1-wq-wqd)/2;
        W = [W; wx wx wq wqd];   %#ok<AGROW>
    end
end
end

function [ok, worst, Pm, nbad, nv] = certify(sc, base, P, restarts)
%CERTIFY  Common quadratic Lyapunov function over the polytope of closed-loop
%         vertices induced by the sector box and the weight polytope.
if nargin < 4, restarts = 2; end
lim = sc .* base;
K1 = zeros(1,4); K2 = zeros(1,4);
for i = 1:4, [K1(i),K2(i)] = sectorBound(i, lim(i), P); end
W = weightVertices(lim(3), P);

V = {};
for r = 1:size(W,1)
    for m = 0:15
        b = bitget(m, 1:4);
        k = K1.*(1-b) + K2.*b;
        V{end+1} = P.A + P.B*(W(r,:).*k); %#ok<AGROW>
    end
end
nv   = numel(V);
nbad = sum(cellfun(@(M) ~all(real(eig(M)) < 0), V));
if nbad > 0, ok = false; worst = inf; Pm = []; return; end

[worst, Pm] = commonP(V, restarts);
ok = worst < -1e-7;
end

function [best, Pbest] = commonP(V, restarts)
%COMMONP  min_P max_v lam_max(Av'P+PAv) s.t. P>0, trace(P)=1.
%   Convex. Solved by projected subgradient (no toolbox required); the returned
%   P is then VERIFIED exactly by an eigenvalue check in certify().
n = 4; best = inf; Pbest = eye(n)/n;
for r = 1:restarts
    if r == 1, Pk = eye(n)/n;
    else,      M = randn(n); Pk = M*M.'; Pk = Pk/trace(Pk); end
    for t = 1:2500
        lam = -inf; u = []; Mw = [];
        for j = 1:numel(V)
            S = V{j}.'*Pk + Pk*V{j}; S = (S+S.')/2;
            [Uv,Dv] = eig(S); [d,ix] = max(diag(Dv));
            if d > lam, lam = d; u = Uv(:,ix); Mw = V{j}; end
        end
        if lam < best, best = lam; Pbest = Pk; end
        G = Mw*(u*u.') + (u*u.')*Mw.'; G = (G+G.')/2;
        Pk = Pk - (0.5/sqrt(t)) * G/max(norm(G),1e-12);
        [Uv,Dv] = eig((Pk+Pk.')/2); d = max(diag(Dv),1e-7); d = d/sum(d);
        Pk = Uv*diag(d)*Uv.';
    end
end
end
