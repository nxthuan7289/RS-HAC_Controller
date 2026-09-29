function [u, ui, w] = fc_law(X, P, mode)
%FC_LAW  Fuzzy-controller (FC) baseline of Section 4.3, written out for simulation.
%
%   [u, ui, w] = FC_LAW(X, P, mode) has the same four-stage structure as rshac_law:
%     Stage 1  semantization     s_i = sigma_i(x_i)              Eqs. (15)-(18), identical
%     Stage 2  inference         out_i = rule base of fuzzy_ctrl_v2.fis (Fig. 7 line)
%     Stage 3  de-semantization  u_i = 4 u_max out_i + 2 u_min   = u_max (4 out_i - 2)
%     Stage 4  weighted sum      u = sum_i w_i(|q|) u_i          Eqs. (20)-(22), identical
%
%   Semantic inputs outside [0.25, 0.75]. The FC inputs are declared over [0.25, 0.75]
%   (the range [th(1-a), th(1+a)] of the SQSM tables), and the initial states of
%   Experiment 1 lie outside it (s_q(10 deg) = 0.80). The paper's FC clips each semantic
%   input to [0.25, 0.75] before inference (Section 4.3), as the four 'Saturate
%   [0.25, 0.75]' blocks of simulation/cartPend_HAC.slx do. Three modes are provided:
%
%     mode = 'clip'  s clipped to [0.25, 0.75], out = min(max(s, 0.25), 0.75).
%                    THIS IS THE FC OF THE PAPER (Tables 4, 5 and 7).
%     mode = 'lin'   the straight line of Fig. 7 continued outside [0.25, 0.75], out = s;
%                    the counterpart of the RS-HAC extrapolation (P.oor = 'extrap').
%                    Sensitivity check only; not reported in the paper.
%     mode = 'fis'   evalfis on the unclipped s. evalfis does not clamp inputs outside
%                    the declared range, and at s <= 0 or s >= 1 this rule base returns
%                    out_i = 0 on the driven channel. Sensitivity check only; not
%                    reported in the paper.
%
%   No saturation is applied here: the actuator bound is a property of the plant side and
%   is applied once, in simulate_fixed_ctrl.m, so experiment E4 can vary it without
%   changing the control law.
%
%   Mode 'fis' makes evalfis warn on every out-of-range sample. exp_setup.m switches
%   fuzzy:general:diagEvalfis_OutOfRangeInput off after showing the behaviour in the log.

persistent fis
if isempty(fis)
    here = fileparts(mfilename('fullpath'));
    fis = readfis(fullfile(here, '..', 'simulation', 'fuzzy_ctrl_v2.fis'));
end

s = zeros(1, 4);
for i = 1:4
    s(i) = fc_semantize(X(i), P.chan{i});
end

switch mode
    case 'lin'
        out = s(:);
    case 'clip'
        out = min(max(s(:), 0.25), 0.75);
    case 'fis'
        out = reshape(evalfis(fis, s), [], 1);
    otherwise
        error('fc_law:mode', 'unknown out-of-range mode "%s"', mode);
end

ui = 4*P.u_max*out + 2*P.u_min;
w  = hac_weights(X(3), P);
u  = w(:).' * ui;
end

% -------------------------------------------------------------------------
function s = fc_semantize(v, c)
%FC_SEMANTIZE  Copy of the local semantize of rshac_law.m, Eqs. (15)-(18).
switch c.sem
    case 'lin', s = (v - c.par(1)) / (c.par(2) - c.par(1));
    case 'igs', s = 1 ./ (1 + exp(-c.par*v));
    otherwise,  error('fc_law:sem', 'unknown semantization "%s"', c.sem);
end
end
