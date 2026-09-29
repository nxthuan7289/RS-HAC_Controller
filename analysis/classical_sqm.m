function v = classical_sqm(term, alpha, theta)
%CLASSICAL_SQM  Classical hedge-algebra SQM of one linguistic term, Eqs. (2)-(5) of the paper.
%
%   v = CLASSICAL_SQM(term, alpha, theta) returns the semantically quantifying
%   mapping of a term of the hedge algebra of Section 2.1 of the paper: generators
%   g- ("small") and g+ ("big"), written '-' and '+' below,, the neutral constant W, and the two hedges
%   V ("very", positive) and L ("little", negative).
%
%   TERM is a char vector read left to right as nested hedges, ending in the
%   generator:
%     'W'          neutral
%     '-'  '+'     c-, c+
%     'L-'         little small           'VL-'   very little small
%     'V+'         very big               'VVL+'  very very little big   ...
%
%   Conventions -- the ones under which this function reproduces the paper's
%   Table 1 exactly (checked in check_sqsm_properties.m):
%     fm(c-) = theta,  fm(c+) = 1 - theta,  mu(L) = alpha,  mu(V) = beta = 1 - alpha
%     fm(h y) = mu(h) * fm(y)
%     Sign(c-) = -1,  Sign(c+) = +1
%     V is positive w.r.t. every hedge :  Sign(V y) = +Sign(y)
%     L is negative w.r.t. every hedge :  Sign(L y) = -Sign(y)
%     Eqs. (2)-(4): v(W) = theta, v(c-) = theta - alpha*fm(c-), v(c+) = theta + alpha*fm(c+)
%     Eq. (5)     : v(h y) = v(y) + Sign(h y) * [ fm(h y) - omega(h y) * fm(h y) ]
%                   omega(h y) = 0.5 * [1 + Sign(h y) * Sign(V h y) * (beta - alpha)]
%   With one positive and one negative hedge the sum in Eq. (5) has a single term.
%
%   Used to test Proposition 2 (agreement of SQSM with the classical SQM).

term = char(term);
if ~(alpha > 0 && alpha < 1) || ~(theta > 0 && theta < 1)
    error('classical_sqm:range', 'alpha and theta must lie in the open interval (0,1).');
end
beta = 1 - alpha;

if strcmp(term, 'W')
    v = theta;
    return
end

switch term(end)
    case '-', v = theta - alpha*theta;       fm = theta;       s = -1;
    case '+', v = theta + alpha*(1 - theta); fm = 1 - theta;   s = +1;
    otherwise
        error('classical_sqm:term', 'term must be ''W'' or end in ''-''/''+'', got "%s".', term);
end

hedges = term(1:end-1);
for idx = numel(hedges):-1:1                 % innermost hedge first
    switch hedges(idx)
        case 'V', mu = beta;   s = +s;        % V positive w.r.t. everything
        case 'L', mu = alpha;  s = -s;        % L negative w.r.t. everything
        otherwise
            error('classical_sqm:hedge', 'unknown hedge "%s" in "%s".', hedges(idx), term);
    end
    fm    = mu*fm;
    sV    = s;                                % Sign(V h y) = Sign(h y)
    omega = 0.5*(1 + s*sV*(beta - alpha));
    v     = v + s*(fm - omega*fm);
end
end
