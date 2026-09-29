function [u, ui, w] = rshac_law(X, P)
%RSHAC_LAW  Explicit closed-form RS-HAC control law (Algorithm 2, Sections 3.5 and 4.2).
%
%   [u, ui, w] = RSHAC_LAW(X, P) evaluates the RS-HAC output for the state
%   X = [x; xdot; q; qdot] using the design P returned by rshac_params.
%
%   Two implementation conventions are read from P (defaults in rshac_params):
%     P.oor       'extrap' (as implemented) | 'clip' | 'haend' -- rule of the SQSM
%                 lookup outside [v(1), v(n)]; see sqsm_lookup below.
%     P.saturate  true -> |u| <= u_max, the explicit saturation of the Simulink
%                 model and the firmware. ui is always returned unsaturated.
%
%   This explicit reference implementation corresponds to the plain MATLAB
%   statement in ../simulation/my_HAC_improved.m. Compare its parameters and
%   lookup convention with the firmware before treating the two as identical.
%
%   Stage 1  semantization     s_i = sigma_i(x_i)           Eqs. (15)-(18)
%   Stage 2  inference         u_is = PWL_i(s_i)            Eq. (11) breakpoints
%   Stage 3  de-semantization  u_i  = u_is*(umax-umin)+umin Eq. (19)
%   Stage 4  weighted sum      u    = sum_i w_i(|q|) u_i    Eqs. (20)-(23)
%
%   Each u_i is a MONOTONE STATIC map of a SINGLE state, so the closed loop is a
%   Lur'e system: linear plant + decoupled sector-bounded nonlinearities. That
%   structure is what makes the stability certificate in rshac_stability.m
%   possible without altering the controller.

nch = numel(P.chan);
ui  = zeros(nch,1);
oor = 'extrap';
if isfield(P, 'oor'), oor = P.oor; end

for i = 1:nch
    c  = P.chan{i};
    s  = semantize(X(i), c);                              % stage 1
    bp = sqsm_values(c.n, c.a_i, c.th_i);                 % state breakpoints
    tb = sqsm_values(c.n, c.a_u, c.th_u);                 % controller table
    us = sqsm_lookup(bp, tb, s, oor);                     % stage 2
    ui(i) = us*(P.u_max - P.u_min) + P.u_min;             % stage 3
end

w = rshac_weights(X(3), P);                               % stage 4
u = w(:).' * ui;
if isfield(P, 'saturate') && P.saturate
    u = min(max(u, P.u_min), P.u_max);                    % explicit saturation
end
end

% -------------------------------------------------------------------------
function us = sqsm_lookup(bp, tb, s, oor)
%SQSM_LOOKUP  Stage 2 piecewise-linear inference with a selectable rule outside
%   the table range [bp(1), bp(end)] = [th(1-a), th(1+a)] (Proposition 1).
switch oor
    case 'extrap'   % as implemented: interp1 'extrap' / look1_binlx
        us = interp1(bp, tb, s, 'linear', 'extrap');
    case 'clip'     % hold the end values
        us = interp1(bp, tb, min(max(s, bp(1)), bp(end)), 'linear');
    case 'haend'    % hedge-algebra constants 0 -> 0 and 1 -> 1 (C = {0, W, 1})
        if bp(1) <= 0 || bp(end) >= 1 || tb(1) <= 0 || tb(end) >= 1
            error('rshac_law:haend', 'haend needs both tables strictly inside (0,1).');
        end
        us = interp1([0 bp 1], [0 tb 1], min(max(s, 0), 1), 'linear');
    otherwise
        error('rshac_law:oor', 'unknown out-of-range rule "%s"', oor);
end
end

% -------------------------------------------------------------------------
function s = semantize(v, c)
%SEMANTIZE  Stage 1. Linear normalization (Eqs. 15-16) or IGS (Eqs. 17-18).
switch c.sem
    case 'lin'
        s = (v - c.par(1)) / (c.par(2) - c.par(1));
    case 'igs'
        s = 1 ./ (1 + exp(-c.par*v));                     % c = 0 (symmetry)
    otherwise
        error('rshac_law:sem','unknown semantization "%s"', c.sem);
end
end

% -------------------------------------------------------------------------
function w = rshac_weights(q, P)
%RSHAC_WEIGHTS  Stage 4, Eqs. (20)-(22). Order: [x xd q qd].
%
%   NOTE: as printed (and as coded in my_HAC_improved.m:54-69) the weight vector
%   is DISCONTINUOUS at |q| = l1: Eq. (20) gives [.25 .25 .25 .25] while the
%   limit of Eq. (21) from above gives [.1875 .1875 .25 .375]. The jump is
%   bounded and is reproduced faithfully here; the stability analysis covers it
%   by taking the convex hull of every reachable weight vector.
aq = abs(q);
if aq <= P.l1
    w = [0.25 0.25 0.25 0.25];
elseif aq >= P.l2
    w = [0 0 1 0];
else
    wq  = 0.25 + (aq - P.l1)*(1 - 0.25)/(P.l2 - P.l1);
    wqd = (1 - wq)/2;
    wx  = (1 - wq - wqd)/2;
    w   = [wx wx wq wqd];
end
end
