%VERIFY_GAIN_FORMULA  Closed-form small-signal gain of RS-HAC.
%
%   Produces the numbers of Section 3.6 (K_RS, the closed-loop spectrum and the
%   inverse design of Corollary 1) and checks that the relative error between the
%   analytic gain and a numerical derivative of the actual control law is < 1e-9.
%
%   RESULT (proof of Proposition 3 and Corollary 1)
%   -----------------------------------------------------------
%   For |q| <= l1 the weights are constant (w_i = 1/4, Eq. 20), the semantic
%   values sit on the innermost SQSM segment, and RS-HAC reduces EXACTLY to the
%   linear state feedback u = K_RS * X with
%
%       K_i = w_i (u_max - u_min) (alpha_ui/alpha_i)^((n_i-1)/2) ds_i/dx_i|_0
%
%   where  ds/dx|_0 = 1/(x_max - x_min)   for linear semantization, and
%          ds/dx|_0 = a/4                 for IGS (since IGS' = a*IGS*(1-IGS)
%                                         and IGS(0) = 1/2).
%
%   The (alpha_u/alpha)^((n-1)/2) factor is the slope of the innermost segment
%   of the interpolation line and follows from Proposition 1 (geometric spacing
%   of the SQSM breakpoints with ratio alpha).
%
%   Run:  >> verify_gain_formula

clear; clc;
P = rshac_params();
nch = numel(P.chan);

fprintf('=== SQSM tables (Algorithm 1) -- compare with Fig. 5 ===\n');
for i = 1:nch
    c = P.chan{i};
    fprintf('  %-3s n=%d  bp = %s\n', c.name, c.n, mat2str(sqsm_values(c.n,c.a_i,c.th_i),4));
    fprintf('  %-3s      tb = %s\n', '',     mat2str(sqsm_values(c.n,c.a_u,c.th_u),4));
end

%% ---- equilibrium is preserved (symmetry, Proposition 1) ----------------
fprintf('\n=== u_i(0) must be exactly 0 (symmetry of Proposition 1) ===\n');
[u0, ui0] = rshac_law(zeros(4,1), P);
fprintf('  u_i(0) = %s      u(0) = %.3e\n', mat2str(ui0.',3), u0);
assert(max(abs(ui0)) < 1e-12 && abs(u0) < 1e-12, 'equilibrium not preserved');

%% ---- closed form vs numerical derivative --------------------------------
fprintf('\n=== small-signal gain: closed form vs numerical derivative ===\n');
fprintf('  %-4s %10s %14s %14s %12s\n','ch','g0','K_closedform','K_numeric','rel.err');
K_cf = zeros(1,nch);  K_nu = zeros(1,nch);
% Central-difference step. Round-off ~ eps*u_max/h, truncation ~ (a*h)^2/12 on the
% logistic channels. h = 1e-7 gave a relative error of 1.28e-9 on channel xd in
% MATLAB R2022b and failed the 1e-9 check below;
% h = 1e-6 keeps both error sources near 1e-10.
h = 1e-6;
for i = 1:nch
    c    = P.chan{i};
    g0   = (c.a_u/c.a_i)^((c.n-1)/2) * (c.th_u/c.th_i);     % innermost slope
    switch c.sem
        case 'lin', ds = 1/(c.par(2)-c.par(1));
        case 'igs', ds = c.par/4;
    end
    K_cf(i) = 0.25*(P.u_max-P.u_min)*g0*ds;

    Xp = zeros(4,1); Xp(i) =  h;   [~, uip] = rshac_law(Xp, P);
    Xm = zeros(4,1); Xm(i) = -h;   [~, uim] = rshac_law(Xm, P);
    K_nu(i) = 0.25*(uip(i) - uim(i))/(2*h);

    re = abs(K_cf(i)-K_nu(i))/abs(K_nu(i));
    fprintf('  %-4s %10.4f %14.4f %14.4f %12.2e\n', c.name, g0, K_cf(i), K_nu(i), re);
    assert(re < 1e-9, 'closed-form gain does not match the control law for channel %s', c.name);
end

%% ---- Proposition 3: local exponential stability -------------------------
fprintf('\n=== Proposition 3: closed-loop spectrum ===\n');
Kl = -lqr_gain();                         % Eq. (25): Kl = lqr(A, B, Q, R), u = -Kl*X
if exist('lqr', 'file')                   % check the stored gain against Eq. (24)
    Kc = lqr(P.A, P.B, diag([4 0.1 11 0.2]), 0.02);
    assert(max(abs(Kc - Kl)./abs(Kl)) < 1e-8, 'lqr_gain.m does not match Eq. (24)');
    fprintf('  lqr(A, B, Q, R) of Eq. (24) reproduces lqr_gain.m (max rel. dev. %.1e)\n', max(abs(Kc - Kl)./abs(Kl)));
end
fprintf('  K_RSHAC (u = +K X) = %s\n', mat2str(K_cf,5));
fprintf('  K_LQR   (u = +K X) = %s\n', mat2str(-Kl,5));
fprintf('  per-channel ratio  = %s\n', mat2str(K_cf./(-Kl),3));

ev_ol = eig(P.A);
ev_rs = eig(P.A + P.B*K_cf);
ev_lq = eig(P.A + P.B*(-Kl));
fprintf('  open loop : %s\n', mat2str(sort(ev_ol).',4));
fprintf('  RS-HAC    : %s   Hurwitz = %d\n', mat2str(sort(ev_rs).',4), all(real(ev_rs)<0));
fprintf('  LQR       : %s   Hurwitz = %d\n', mat2str(sort(ev_lq).',4), all(real(ev_lq)<0));
assert(all(real(ev_rs) < 0), 'Proposition 3 fails: A+B*K_RS is not Hurwitz');

%% ---- inverse design: pick alpha_u to place a desired gain ---------------
% Answers R2-D/E ("how did you impose the performance specifications?") and
% R3-10 ("tuning is subjective"): alpha_u is no longer a trial-and-error knob.
fprintf('\n=== inverse design: alpha_u that realises a target gain K* ===\n');
fprintf('  alpha_ui = alpha_i * ( K*_i / (w_i (umax-umin) ds_i) )^(2/(n_i-1))\n');
Kstar = -Kl;                                   % target = the LQR gain
fprintf('  %-4s %12s %12s %10s\n','ch','K*','alpha_u*','in (0,1)?');
for i = 1:nch
    c = P.chan{i};
    switch c.sem
        case 'lin', ds = 1/(c.par(2)-c.par(1));
        case 'igs', ds = c.par/4;
    end
    au = c.a_i * ( Kstar(i) / (0.25*(P.u_max-P.u_min)*ds) )^(2/(c.n-1));
    fprintf('  %-4s %12.4f %12.4f %10s\n', c.name, Kstar(i), au, ...
            string(au>0 && au<1));
end

fprintf('\nAll assertions passed.\n');
