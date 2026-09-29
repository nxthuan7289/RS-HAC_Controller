%CHECK_SQSM_PROPERTIES  Numerical checks of the SQSM properties used in the paper.
%
%   The script evaluates the formulas over representative parameter grids and
%   reports the corresponding numerical checks.
%
%   Run from this folder:   >> check_sqsm_properties
%   Log:                    logs/check_sqsm_properties.txt

clear; clc;
here   = fileparts(mfilename('fullpath'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'check_sqsm_properties.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('check_sqsm_properties   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

P   = rshac_params();
tol = 1e-12;
du  = P.u_max - P.u_min;

nGrid = 3:2:15;
aGrid = [0.10 0.25 0.35 0.50 0.65 0.725 0.80 0.95];
tGrid = [0.30 0.50 0.70];

%% ---- Proposition 1: order, range and symmetry ------------------------------------
fprintf('=== Proposition 1 (order, range, symmetry): n = 3..15 odd, %d alphas, %d thetas ===\n', numel(aGrid), numel(tGrid));
nCase = 0;
for n = nGrid
    MSI = (n+1)/2;
    for a = aGrid
        for th = tGrid
            v = sqsm_values(n, a, th);
            assert(all(diff(v) > 0), 'Proposition 1 (order) fails: n=%d a=%g th=%g', n, a, th);
            assert(abs(v(1) - th*(1-a)) < tol && abs(v(end) - th*(1+a)) < tol, ...
                   'Proposition 1 (range) fails: n=%d a=%g th=%g', n, a, th);
            j = 1:MSI-1;
            assert(max(abs(v(MSI+j) + v(MSI-j) - 2*th)) < tol, 'Proposition 1 (symmetry) fails: n=%d a=%g th=%g', n, a, th);
            nCase = nCase + 1;
        end
    end
end
fprintf('  strict order, range [th(1-a), th(1+a)], symmetry about v(MSI) = th : %d cases OK\n\n', nCase);

%% ---- T1: gaps between adjacent labels (Proposition 1) ----------------------------
fprintf('=== T1: gaps between adjacent labels (Proposition 1) ===\n');
v = sqsm_values(5, 0.5, 0.5);
fprintf('  n=5, a=th=0.5: v = %s\n', mat2str(v, 4));
fprintf('  innermost gap v(3)-v(2) = %.4f | outer-gap formula th*a^k*(1-a) at k=2: %.4f | neutral gap th*a^(MSI-1): %.4f\n', ...
        v(3) - v(2), 0.5*0.5^2*(1 - 0.5), 0.5*0.5^2);
nCase = 0;
for n = nGrid
    MSI = (n+1)/2;
    for a = aGrid
        for th = tGrid
            dv = diff(sqsm_values(n, a, th));
            dv = dv(1:MSI-1);                                   % k = 1 .. MSI-1 (lower half)
            k  = 1:MSI-2;
            if ~isempty(k)
                assert(max(abs(dv(k) - th*a.^k*(1-a))) < tol, 'gap law (outer) fails: n=%d a=%g th=%g', n, a, th);
                assert(abs(dv(MSI-1)/dv(MSI-2) - a/(1-a)) < 1e-9, 'innermost ratio fails: n=%d a=%g', n, a);
            end
            assert(abs(dv(MSI-1) - th*a^(MSI-1)) < tol, 'gap law (innermost) fails: n=%d a=%g th=%g', n, a, th);
            if MSI >= 4
                assert(max(abs(dv(2:MSI-2)./dv(1:MSI-3) - a)) < 1e-9, 'outer ratio fails: n=%d a=%g', n, a);
            end
            if n >= 5, gmin = th*a^(MSI-2)*min(a, 1-a); else, gmin = th*a; end
            assert(abs(min(dv) - gmin) < tol, 'minimum gap fails: n=%d a=%g th=%g', n, a, th);
            assert(th*a^(MSI-1)*(1-a) <= min(dv) + tol, 'th*a^(MSI-1)*(1-a) is not a lower bound');
            nCase = nCase + 1;
        end
    end
end
fprintf('  Dv(k) = th*a^k*(1-a) for k <= MSI-2, Dv(MSI-1) = th*a^(MSI-1), ratio a (a/(1-a) at the last step),\n');
fprintf('  minimum gap th*a^(MSI-2)*min(a,1-a) (n>=5); th*a^(MSI-1)*(1-a) is only a lower bound: %d cases OK\n\n', nCase);

%% ---- T2: segment slopes of the Table 3 control lines ------------------------------
fprintf('=== T2: segment slopes of the four control lines (k = 1 outermost ... MSI-1 innermost) ===\n');
for i = 1:4
    c   = P.chan{i};
    MSI = (c.n+1)/2;
    bp  = sqsm_values(c.n, c.a_i, c.th_i);
    tb  = sqsm_values(c.n, c.a_u, c.th_u);
    g   = diff(tb)./diff(bp);
    assert(max(abs(g - fliplr(g))) < 1e-12, 'slopes not symmetric for %s', c.name);
    gl  = g(1:MSI-1);
    k   = 1:MSI-2;
    r   = c.a_u/c.a_i;
    gCF = [(c.th_u/c.th_i)*((1-c.a_u)/(1-c.a_i))*r.^k, (c.th_u/c.th_i)*r^(MSI-1)];
    assert(max(abs(gl - gCF)) < 1e-12, 'slope formula of Lemma 1 fails for %s', c.name);
    if c.a_u > c.a_i
        assert(all(diff(gl) > 0)); shape = 'compressive: steepest at equilibrium';
    elseif c.a_u < c.a_i
        assert(all(diff(gl) < 0)); shape = 'expansive: steepest at the table ends';
    else
        shape = 'straight line';
    end
    fprintf('  %-3s n=%d a=%.2f a_u=%.3f  slopes %-26s -> %s\n', c.name, c.n, c.a_i, c.a_u, mat2str(gl, 4), shape);
end
fprintf('\n');

%% ---- T3: innermost slope versus the number of labels ------------------------------
fprintf('=== T3: innermost slope versus n with alpha, alpha_u held fixed ===\n');
nn = [3 5 7 9 11 15];
fprintf('  %-34s', 'n'); fprintf('%9d', nn); fprintf('\n');
for ci = [4 1]
    c  = P.chan{ci};
    sl = zeros(1, numel(nn));
    for m = 1:numel(nn)
        n   = nn(m);
        MSI = (n+1)/2;
        bp  = sqsm_values(n, c.a_i, c.th_i);
        tb  = sqsm_values(n, c.a_u, c.th_u);
        sl(m) = (tb(MSI+1) - tb(MSI))/(bp(MSI+1) - bp(MSI));
        assert(abs(sl(m) - (c.th_u/c.th_i)*(c.a_u/c.a_i)^((n-1)/2)) < 1e-9*max(1, sl(m)));
    end
    fprintf('  %-34s', sprintf('%s (a=%.2f, a_u=%.3f) innermost slope', c.name, c.a_i, c.a_u)); fprintf('%9.3f', sl); fprintf('\n');
    fprintf('  %-34s', '   -> small-signal gain K'); fprintf('%9.3f', 0.25*du*sl*semDeriv0(c)); fprintf('\n');
end
fprintf('  (changing n at fixed alpha changes the gain by (a_u/a) per added label pair, not only the resolution)\n\n');

%% ---- T4: logistic, tanh and other sigmoids against axiom (S5) ---------------------
% (S5): logit(sigma(x+d)) - logit(sigma(x)) depends on d only (the logistic map of Eq. (7) has it).
% Test it directly through the odds-doubling distance D(x), the crisp step after which the
% semantic odds sigma/(1-sigma) have doubled. (S5) <=> D(x) is the same at every x.
% All maps are slope-matched at 0 (sigma'(0) = a/4). The log-odds L are written with the
% complement 1 - sigma in closed form so that the far tail (x = 20) is not lost to rounding.
fprintf('=== T4: which sigmoids satisfy (S5)? odds form: logit(sigma(x+d)) - logit(sigma(x)) = psi(d) ===\n');
b  = 1.7;  c0 = 0.3;  x = linspace(-10, 10, 20001);
eT = max(abs((1 + tanh(b*(x - c0)))/2 - 1./(1 + exp(-2*b*(x - c0)))));
fprintf('  max |(1+tanh(b(x-c)))/2 - logistic(a=2b, c)| = %.2e   -> normalised tanh IS the logistic map\n', eT);
assert(eT < 1e-14);

a   = 2;
kA  = pi*a/4;                         % arctan:    sigma = 1/2 + atan(kA x)/pi
kP  = a*sqrt(2*pi)/4;                 % probit:    sigma = Phi(kP x)
kS  = a/2;                            % algebraic: sigma = 1/2 + y/(2 sqrt(1+y^2)),  y = kS x
names = {'logistic', 'normalised tanh', 'normalised arctan', 'probit (Gaussian cdf)', 'algebraic sigmoid'};
L = { @(x) a*x, ...
      @(x) log((1 + tanh(a*x/2)).*(1 + exp(a*x))/2), ...                           % 1 - tanh(z) = 2/(1 + e^(2z))
      @(x) log((0.5 + atan(kA*x)/pi)./(atan(1./(kA*x))/pi)), ...                    % x > 0
      @(x) log(erfc(-kP*x/sqrt(2))./erfc(kP*x/sqrt(2))), ...
      @(x) log((0.5 + kS*x./(2*sqrt(1 + (kS*x).^2))) ./ ...
               (1./(2*sqrt(1 + (kS*x).^2).*(sqrt(1 + (kS*x).^2) + kS*x)))) };
sig = { @(x) 1./(1 + exp(-a*x)), @(x) (1 + tanh(a*x/2))/2, @(x) 0.5 + atan(kA*x)/pi, ...
        @(x) 0.5*erfc(-kP*x/sqrt(2)), @(x) 0.5 + kS*x./(2*sqrt(1 + (kS*x).^2)) };

xs    = [0.01 1 2 20];                % 0.01 not 0: the arctan closed form is for x > 0
Dfun  = @oddsDoublingDistance;
h     = 1e-6;
fprintf('  slope-matched at 0 with a = %g;  UGS doubles the odds every ln2/a = %.4f\n', a, log(2)/a);
fprintf('  %-22s %12s %10s %10s %10s %10s   %s\n', 'map', '4sigma''(0)/a', 'D(0.01)', 'D(1)', 'D(2)', 'D(20)', 'asymptotic D(x)');
asym = {'ln2/a (constant)', 'ln2/a (constant)', '~ x', '~ ln2/(kP^2 x)', '~ (sqrt2 - 1) x'};
D = zeros(numel(L), numel(xs));
for m = 1:numel(L)
    for j = 1:numel(xs), D(m, j) = Dfun(L{m}, xs(j)); end
    s0 = (sig{m}(h) - sig{m}(-h))/(2*h);
    fprintf('  %-22s %12.6f %10.4f %10.4f %10.4f %10.4f   %s\n', names{m}, 4*s0/a, D(m, :), asym{m});
    assert(abs(4*s0/a - 1) < 1e-6, 'slope matching failed for %s', names{m});
end
% (S5) holds exactly for logistic and tanh, fails for the other three
assert(all(max(abs(D(1:2, :) - log(2)/a), [], 2) < 1e-9));
assert(all(max(D(3:5, :), [], 2)./min(D(3:5, :), [], 2) > 2));
% ... and fails the way the tail asymptotics predict (checked at x = 20, 5 % tolerance)
fprintf('  x = 20:  arctan D/x = %.4f (-> 1),  algebraic D/x = %.4f (-> %.4f),  probit D*kP^2*x/ln2 = %.4f (-> 1)\n', ...
        D(3, 4)/20, D(5, 4)/20, sqrt(2) - 1, D(4, 4)*kP^2*20/log(2));
assert(abs(D(3, 4)/20 - 1) < 0.05 && abs(D(5, 4)/20/(sqrt(2) - 1) - 1) < 0.05 && abs(D(4, 4)*kP^2*20/log(2) - 1) < 0.05);

% equivalence with the differential form sigma' = a sigma (1 - sigma) (differentiable case)
xg = linspace(-3, 3, 6001);  hx = xg(2) - xg(1);
fprintf('  differential form, max|sigma'' - a sigma(1 - sigma)| on [-3, 3]:');
for m = 1:numel(sig)
    s = sig{m}(xg);  ds = gradient(s, hx);
    short = {'logistic', 'tanh', 'arctan', 'probit', 'algebraic'};
    fprintf('  %s %.1e', short{m}, max(abs(ds(2:end-1) - a*s(2:end-1).*(1 - s(2:end-1)))));
end
fprintf('\n');
fprintf('  semantic odds double every ln2/a:  q -> %.3f deg (a_q = %g),  qd -> %.1f deg/s (a_qd = %g)\n\n', ...
        rad2deg(log(2)/P.chan{3}.par), P.chan{3}.par, rad2deg(log(2)/P.chan{4}.par), P.chan{4}.par);

%% ---- T5 / Proposition 2: SQSM versus the classical SQM ----------------------------
fprintf('=== T5 / Proposition 2: SQSM versus the classical SQM of Eqs. (2)-(5) ===\n');
lab7 = {'V-', '-', 'L-', 'W', 'L+', '+', 'V+'};                 % VS S LS W LB B VB
nTab = 0;
for a = aGrid
    for th = tGrid
        got = cellfun(@(t) classical_sqm(t, a, th), lab7);
        ref = [th*(1-a)^2, th*(1-a), th*(1-a+a^2), th, th+a*(1-th)*(1-a), th+a*(1-th), th+a*(1-th)*(2-a)];
        assert(max(abs(got - ref)) < tol, 'classical_sqm does not reproduce Table 1 (a=%g th=%g)', a, th);
        nTab = nTab + 1;
    end
end
fprintf('  (a) classical_sqm reproduces Table 1 for %d (alpha, theta) pairs\n', nTab);

a = 0.5;  th = 0.5;
legacy = [th*(1-a), th*(1-a+a^2), th, th+a*(1-th)*(1-a), th+a*(1-th)];
got    = cellfun(@(t) classical_sqm(t, a, th), {'-', 'L-', 'W', 'L+', '+'});
assert(max(abs(got - legacy)) < tol);
fprintf('  (b) legacy SQMs_state (init_HAC_simulation.m:125-129) = SQM of {c-, Lc-, W, Lc+, c+}\n');

v7sqm  = cellfun(@(t) classical_sqm(t, 0.5, 0.5), lab7);
v7sqsm = sqsm_values(7, 0.5, 0.5);
fprintf('  (c) the 7 labels of Table 1 at a=th=0.5:  SQM  = %s\n', mat2str(v7sqm, 4));
fprintf('                                            SQSM = %s  -> differ (max %.4f)\n', ...
        mat2str(v7sqsm, 4), max(abs(v7sqm - v7sqsm)));
assert(max(abs(v7sqm - v7sqsm)) > 0.1);

for n = nGrid
    vs = cellfun(@(t) classical_sqm(t, 0.5, 0.5), sqsmChainTerms(n));
    assert(max(abs(vs - sqsm_values(n, 0.5, 0.5))) < tol, 'chain identity fails for n = %d', n);
end
fprintf('  (d) a=th=0.5: SQSM(n) = SQM of the chain {%s} (shown for n=9), for every odd n = 3..15\n', ...
        strjoin(sqsmChainTerms(9), ', '));

fprintf('  (e) theta = 0.5, n = 9, alpha ~= 0.5: first value agrees, the chains separate from the second label\n');
for a = [0.35 0.65 0.80]
    vs = cellfun(@(t) classical_sqm(t, a, 0.5), sqsmChainTerms(9));
    vq = sqsm_values(9, a, 0.5);
    gs = diff(vs(1:5));  gq = diff(vq(1:5));
    rs = gs(2:3)./gs(1:2);  rq = gq(2:3)./gq(1:2);
    fprintf('      a=%.2f  SQM %s | SQSM %s | gap ratios SQM %s, SQSM %s\n', ...
            a, mat2str(vs(1:4), 4), mat2str(vq(1:4), 4), mat2str(rs, 3), mat2str(rq, 3));
    assert(abs(vs(1) - vq(1)) < tol && abs(vs(2) - vq(2)) > 1e-4);
    assert(max(abs(rs - (1-a))) < 1e-9 && max(abs(rq - a)) < 1e-9);
end

fine  = (1:19)/20;
hits5 = zeros(0, 2);  hits3 = zeros(0, 2);
for a = fine
    for th = fine
        if max(abs(cellfun(@(t) classical_sqm(t, a, th), sqsmChainTerms(5)) - sqsm_values(5, a, th))) < 1e-12
            hits5(end+1, :) = [a th]; %#ok<AGROW>
        end
        if max(abs(cellfun(@(t) classical_sqm(t, a, th), sqsmChainTerms(3)) - sqsm_values(3, a, th))) < 1e-12
            hits3(end+1, :) = [a th]; %#ok<AGROW>
        end
    end
end
assert(size(hits5, 1) == 1 && all(abs(hits5 - 0.5) < 1e-12));
assert(size(hits3, 1) == numel(fine) && all(abs(hits3(:, 2) - 0.5) < 1e-12));
fprintf('  (f) on a 19x19 grid: n=5 coincides only at a=th=0.5; n=3 coincides iff th=0.5 (any a)\n\n');

%% ---- T6: IGS / IGDS round trip ----------------------------------------------------
fprintf('=== T6: IGS / IGDS round trip, Eqs. (7)-(8) ===\n');
a  = 8;  c0 = 0.1;  x = linspace(-1, 1, 2001);
s  = 1./(1 + exp(-a*(x - c0)));
xPrinted = (1/a)*log((1 - s)./s) + c0;          % sign-reversed inverse (submitted version of Eq. (8))
xCorrect = c0 + (1/a)*log(s./(1 - s));
fprintf('  sign-reversed Eq. (8): max|IGDS(IGS(x)) - x| = %.3e, and IGDS(IGS(x)) = 2c - x to %.1e\n', ...
        max(abs(xPrinted - x)), max(abs(xPrinted - (2*c0 - x))));
fprintf('  Eq. (8)             : max|IGDS(IGS(x)) - x| = %.3e\n', max(abs(xCorrect - x)));
assert(max(abs(xCorrect - x)) < 1e-9 && max(abs(xPrinted - x)) > 0.1 && max(abs(xPrinted - (2*c0 - x))) < 1e-9);
fprintf('  a = 0 gives IGS(x) = 0.5 for every x (not a bijection) and Eq. (8) divides by zero -> require a > 0\n\n');

%% ---- T7: weights, Eqs. (20)-(22) --------------------------------------------------
fprintf('=== T7: adaptive weights, Eqs. (20)-(22) ===\n');
[~, ~, wA] = rshac_law([0; 0; P.l1; 0], P);
[~, ~, wB] = rshac_law([0; 0; P.l1*(1 + 1e-9); 0], P);
fprintf('  w = [wx wxd wq wqd] at |q| = l1 (Eq. 20): %s ; just above l1 (Eq. 21): %s ; jump %s\n', ...
        mat2str(wA, 4), mat2str(wB, 4), mat2str(wB - wA, 4));
assert(max(abs((wB - wA) - [-0.0625 -0.0625 0 0.125])) < 1e-6);
q = -deg2rad(10);
wqPrinted = 0.25 + (q - P.l1)*(1 - 0.25)/(P.l2 - P.l1);     % Eq. (21) as printed, with q
[~, ~, wAbs] = rshac_law([0; 0; q; 0], P);
fprintf('  q = -10 deg: Eq. (21) as printed gives w_q = %.4f ; with |q| (as coded) w_q = %.4f\n\n', wqPrinted, wAbs(3));
assert(abs(wqPrinted) < 1e-12 && abs(wAbs(3) - 1/3) < 1e-12);

%% ---- T8: neighbourhood N of Proposition 3 -----------------------------------------
fprintf('=== T8: neighbourhood N on which every lookup stays on its innermost segment ===\n');
Nlim = zeros(1, 4);  g0 = zeros(1, 4);  tbEnd = zeros(1, 4);  K = zeros(1, 4);
for i = 1:4
    c   = P.chan{i};
    MSI = (c.n+1)/2;
    bp  = sqsm_values(c.n, c.a_i, c.th_i);
    tb  = sqsm_values(c.n, c.a_u, c.th_u);
    g0(i)    = (tb(MSI+1) - tb(MSI))/(bp(MSI+1) - bp(MSI));
    Nlim(i)  = semInverse(c, bp(MSI+1));
    tbEnd(i) = semInverse(c, bp(end));
    K(i)     = 0.25*du*g0(i)*semDeriv0(c);
end
Nlim(3) = min(Nlim(3), P.l1);                     % weights also constant
fprintf('  N          : |x| <= %.4f m, |xd| <= %.3f m/s, |q| <= %.3f deg, |qd| <= %.2f deg/s\n', ...
        Nlim(1), Nlim(2), rad2deg(Nlim(3)), rad2deg(Nlim(4)));
fprintf('  tables end : |x| =  %.4f m, |xd| =  %.3f m/s, |q| =  %.3f deg, |qd| =  %.2f deg/s\n', ...
        tbEnd(1), tbEnd(2), rad2deg(tbEnd(3)), rad2deg(tbEnd(4)));
fprintf('  K_RS = %s\n', mat2str(K, 5));

rng(1);  M = 4000;  errSmooth = 0;
for j = 1:M
    X    = (2*rand(4, 1) - 1).*Nlim(:)*(1 - 1e-9);
    uLaw = rshac_law(X, P);
    uCF  = 0;
    for i = 1:4, uCF = uCF + 0.25*du*g0(i)*(semantize0(P.chan{i}, X(i)) - 0.5); end
    errSmooth = max(errSmooth, abs(uLaw - uCF));
end
fprintf('  inside N the law equals u = sum_i du/4*g0_i*(s_i(x_i) - 1/2): max error over %d random states = %.2e\n', M, errSmooth);
assert(errSmooth < 1e-10);

relLin = zeros(1, 4);
for i = 1:4
    X = zeros(4, 1);  X(i) = Nlim(i);
    [~, ui] = rshac_law(X, P);
    relLin(i) = abs(0.25*ui(i) - K(i)*X(i))/abs(K(i)*X(i));
end
fprintf('  |u_i - K_i x_i|/|K_i x_i| at the edge of N:  x %.1e,  xd %.1e,  q %.2f %%,  qd %.2f %%\n', ...
        relLin(1), relLin(2), 100*relLin(3), 100*relLin(4));
fprintf('  -> exactly linear in x, xd only; smooth with Jacobian K_RS at 0 in q, qd (Proposition 3 by the indirect method)\n');
assert(relLin(1) < 1e-10 && relLin(2) < 1e-10 && relLin(3) > 1e-3 && relLin(4) > 1e-4);

for i = 1:4
    X = zeros(4, 1);  X(i) = 1.5*Nlim(i);
    uLaw = rshac_law(X, P);
    uCF  = 0.25*du*g0(i)*(semantize0(P.chan{i}, X(i)) - 0.5);
    assert(abs(uLaw - uCF) > 1e-6, 'lookup still on the innermost segment outside N (channel %d)', i);
end
X    = [0; 0; 0.9*P.l1; 0];
uLaw = rshac_law(X, P);
uCF  = 0.25*du*g0(3)*(semantize0(P.chan{3}, X(3)) - 0.5);
fprintf('  the region |q| <= l1 is larger than N: at q = %.2f deg the q-lookup has left its innermost segment (|dev| = %.3f m/s^2)\n\n', ...
        rad2deg(X(3)), abs(uLaw - uCF));
assert(abs(uLaw - uCF) > 1e-6);

%% ---- G1: bound on |u| for the three out-of-range rules ----------------------------
fprintf('=== G1: bound on |u_i| for the three out-of-range rules ===\n');
aU    = cellfun(@(c) c.a_u, P.chan).';
aI    = cellfun(@(c) c.a_i, P.chan).';
modes = {'extrap', 'clip', 'haend'};
kapCF = {aU.*(1 + (1 - aU)./aI), aU, ones(1, 4)};
for m = 1:numel(modes)
    Pm = P;  Pm.oor = modes{m};
    kap = zeros(1, 4);
    for i = 1:4
        c = P.chan{i};
        if strcmp(c.sem, 'lin'), xe = c.par(2); else, xe = 40/c.par; end
        for sgn = [-1 1]
            X = zeros(4, 1);  X(i) = sgn*xe;
            [~, ui] = rshac_law(X, Pm);
            kap(i) = max(kap(i), abs(ui(i))/P.u_max);
        end
    end
    assert(max(abs(kap - kapCF{m})) < 1e-9, 'kappa mismatch for rule %s', modes{m});
    fprintf('  %-7s |u_i|/u_max at the ends of the semantic domain: %-26s -> kappa = %.4f\n', ...
            modes{m}, mat2str(kap, 4), max(kap));
end
aa = (1:19)/20;
[AU, AI] = meshgrid(aa, aa);
KAP = AU.*(1 + (1 - AU)./AI);
assert(isequal(KAP <= 1 + 1e-12, AU <= AI + 1e-12));
fprintf('  extrapolation: kappa_i = a_u(1 + (1-a_u)/a) <= 1 exactly when a_u,i <= a_i (19x19 grid)\n\n');

%% ---- C2: realisable small-signal gains and the inverse design ---------------------
fprintf('=== C2: realisable small-signal gains and the inverse design ===\n');
Kmax   = zeros(1, 4);
auStar = zeros(1, 4);
Klqr   = lqr_gain();                             % Eq. (25), u = +K*X
for i = 1:4
    c = P.chan{i};
    Kmax(i)   = 0.25*du*(c.th_u/c.th_i)*c.a_i^(-(c.n-1)/2)*semDeriv0(c);
    auStar(i) = c.a_i*(Klqr(i)/(0.25*du*(c.th_u/c.th_i)*semDeriv0(c)))^(2/(c.n-1));
end
fprintf('  K_max (a_u -> 1)     = %s\n', mat2str(Kmax, 4));
fprintf('  K_LQR / K_max        = %s\n', mat2str(Klqr./Kmax, 3));
fprintf('  a_u for K* = K_LQR   = %s   (all in (0,1): %d)\n', mat2str(auStar, 3), all(auStar > 0 & auStar < 1));
assert(all(auStar > 0 & auStar < 1));
P2 = P;
for i = 1:4, P2.chan{i}.a_u = auStar(i); end
h = 1e-7;  Knum = zeros(1, 4);
for i = 1:4
    Xp = zeros(4, 1);  Xp(i) = h;
    Knum(i) = (rshac_law(Xp, P2) - rshac_law(-Xp, P2))/(2*h);
end
fprintf('  numerical gain of the redesigned RS-HAC = %s\n', mat2str(Knum, 5));
assert(max(abs(Knum - Klqr)./Klqr) < 1e-6);

fprintf('\nALL CHECKS PASSED\n');
diary off;

%% ---- local functions --------------------------------------------------------------
function d = semDeriv0(c)
%SEMDERIV0  ds/dx at x = 0 of the semantization of channel c.
switch c.sem
    case 'lin', d = 1/(c.par(2) - c.par(1));
    case 'igs', d = c.par/4;
end
end

function s = semantize0(c, v)
%SEMANTIZE0  Semantization of channel c (Eqs. 15-18, c = 0).
switch c.sem
    case 'lin', s = (v - c.par(1))/(c.par(2) - c.par(1));
    case 'igs', s = 1./(1 + exp(-c.par*v));
end
end

function x = semInverse(c, s)
%SEMINVERSE  Crisp value whose semantic value is s (Eq. (8) for IGS).
switch c.sem
    case 'lin', x = s*(c.par(2) - c.par(1)) + c.par(1);
    case 'igs', x = log(s/(1 - s))/c.par;
end
end

function terms = sqsmChainTerms(n)
%SQSMCHAINTERMS  Terms c-, Lc-, VLc-, V^2Lc-, ..., W, ..., VLc+, Lc+, c+ (n odd), in increasing order.
MSI   = (n+1)/2;
lower = cell(1, MSI-1);
for k = 1:MSI-1
    if k == 1
        lower{k} = '-';
    else
        lower{k} = [repmat('V', 1, k-2) 'L-'];
    end
end
upper = cellfun(@(t) strrep(t, '-', '+'), lower, 'UniformOutput', false);
terms = [lower, {'W'}, fliplr(upper)];
end

function d = oddsDoublingDistance(L, x0)
%ODDSDOUBLINGDISTANCE  Crisp step d > 0 with L(x0 + d) - L(x0) = ln 2, L the log-odds.
%   The bracket grows from 1e-3 until the log-odds gain exceeds ln 2, so the far tail is never
%   evaluated where the complement 1 - sigma underflows.
g  = @(s) L(x0 + s) - L(x0) - log(2);
hi = 1e-3;
while g(hi) < 0
    hi = 2*hi;
    assert(hi < 1e3, 'odds never double within 1e3 of x0 = %g', x0);
end
d = fzero(g, [1e-12, hi]);
end
