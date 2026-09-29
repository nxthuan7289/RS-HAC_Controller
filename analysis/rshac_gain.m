function [K, g0, ds] = rshac_gain(P)
%RSHAC_GAIN  Closed-form small-signal gain of RS-HAC at the origin (u = +K*X).
%
%   [K, g0, ds] = RSHAC_GAIN(P) for the design P returned by rshac_params:
%
%     K_i  = w_i (u_max - u_min) g0_i ds_i,   w_i = 1/4 (Eq. 20)
%     g0_i = (th_u/th_i) (a_u/a_i)^((n_i-1)/2)   innermost slope of the control
%            line (Proposition 1: the gap next to the neutral label is th*a^(MSI-1))
%     ds_i = 1/(x_max - x_min)   linear semantization (Eqs. 15-16)
%          = a/4                 logistic semantization (Eqs. 17-18), since
%                                sigma' = a*sigma*(1-sigma) and sigma(0) = 1/2
%
%   Verified against a numerical derivative of rshac_law in verify_gain_formula.m
%   and check_sqsm_properties.m.

nch = numel(P.chan);
K   = zeros(1, nch);
g0  = zeros(1, nch);
ds  = zeros(1, nch);
for i = 1:nch
    c     = P.chan{i};
    g0(i) = (c.th_u/c.th_i)*(c.a_u/c.a_i)^((c.n - 1)/2);
    switch c.sem
        case 'lin', ds(i) = 1/(c.par(2) - c.par(1));
        case 'igs', ds(i) = c.par/4;
        otherwise,  error('rshac_gain:sem', 'unknown semantization "%s"', c.sem);
    end
    K(i) = 0.25*(P.u_max - P.u_min)*g0(i)*ds(i);
end
end
