function [u, ui, w] = rshac_law_vec(X, P)
%RSHAC_LAW_VEC  Vectorised rshac_law: the RS-HAC output for many states at once.
%
%   [u, ui, w] = RSHAC_LAW_VEC(X, P) with X 4-by-N returns u 1-by-N, ui 4-by-N and
%   w 4-by-N. Same stages, conventions (P.oor, P.saturate) and weights as
%   rshac_law.m; used by the simulation scripts, where rshac_law would be too slow.
%   Equality with rshac_law is asserted in validate_invariant_set.m.

N  = size(X, 2);
ui = zeros(4, N);
oor = 'extrap';
if isfield(P, 'oor'), oor = P.oor; end

for i = 1:4
    c = P.chan{i};
    switch c.sem
        case 'lin', s = (X(i, :) - c.par(1))/(c.par(2) - c.par(1));      % Eqs. (15)-(16)
        case 'igs', s = 1./(1 + exp(-c.par*X(i, :)));                    % Eqs. (17)-(18)
    end
    bp = sqsm_values(c.n, c.a_i, c.th_i);
    tb = sqsm_values(c.n, c.a_u, c.th_u);
    switch oor
        case 'extrap', us = interp1(bp, tb, s, 'linear', 'extrap');
        case 'clip',   us = interp1(bp, tb, min(max(s, bp(1)), bp(end)), 'linear');
        case 'haend',  us = interp1([0 bp 1], [0 tb 1], min(max(s, 0), 1), 'linear');
        otherwise,     error('rshac_law_vec:oor', 'unknown out-of-range rule "%s"', oor);
    end
    ui(i, :) = us*(P.u_max - P.u_min) + P.u_min;                         % Eq. (19)
end

aq  = abs(X(3, :));                                                      % Eqs. (20)-(22)
wq  = 0.25 + (aq - P.l1)*(1 - 0.25)/(P.l2 - P.l1);
wqd = (1 - wq)/2;
wx  = (1 - wq - wqd)/2;
w   = [wx; wx; wq; wqd];
low  = aq <= P.l1;
high = aq >= P.l2;
w(:, low)  = 0.25;
w(:, high) = repmat([0; 0; 1; 0], 1, nnz(high));

u = sum(w.*ui, 1);                                                       % Eq. (23)
if isfield(P, 'saturate') && P.saturate
    u = min(max(u, P.u_min), P.u_max);
end
end
