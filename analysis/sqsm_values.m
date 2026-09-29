function v = sqsm_values(n, alpha, theta)
%SQSM_VALUES  Algorithm 1 of the paper: recursive generation of SQSM values.
%
%   v = SQSM_VALUES(n, alpha, theta) returns the 1-by-n vector of semantically
%   quantifying simplified mapping values for a linguistic variable with n
%   labels, per Eq. (11):
%
%             / theta*(1 - alpha^SIE(x))          , SIE(x) < MSI
%      v(x) = | theta*(1 + alpha^(n+1-SIE(x)))    , SIE(x) > MSI
%             \ theta                             , otherwise
%
%   The lower branch uses SIE(x) < MSI and the upper branch uses SIE(x) > MSI.
%
%   Requires n odd (there is always a neutral label) and alpha, theta in (0,1).
%
%   The loop runs exactly (n-1)/2 times, so table generation has O(n) cost.
%   This is a design-time operation; the values are stored as lookup breakpoints.
%   The per-cycle lookup cost is O(sum_i log n_i); see rshac_law.m.

if mod(n,2) ~= 1
    error('sqsm_values:nOdd','n must be odd (a neutral label always exists), got %d.', n);
end
if ~(alpha > 0 && alpha < 1) || ~(theta > 0 && theta < 1)
    error('sqsm_values:range','alpha and theta must lie in the open interval (0,1).');
end

MSI = (n-1)/2 + 1;              % Eq. (10)
v   = zeros(1,n);
v(MSI) = theta;
for i = 1:(MSI-1)               % exactly (n-1)/2 iterations
    v(i)       = theta*(1 - alpha^i);
    v(n+1-i)   = theta*(1 + alpha^i);
end
end
