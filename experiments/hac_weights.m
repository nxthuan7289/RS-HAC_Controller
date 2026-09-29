function w = hac_weights(q, P)
%HAC_WEIGHTS  Stage-4 adaptive weights, Eqs. (20)-(22). Order [x xd q qd].
%
%   Identical to the local subfunction rshac_weights inside rshac_law.m; kept as a
%   file so that the FC baseline (fc_law.m) blends its channels with exactly the same
%   weights. The discontinuity at |q| = l1 is reproduced faithfully, see rshac_law.m.
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
