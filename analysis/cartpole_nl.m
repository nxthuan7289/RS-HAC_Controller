function dX = cartpole_nl(X, u, P, tau)
%CARTPOLE_NL  Nonlinear cart-pole with cart-acceleration input, Eq. (12).
%
%   dX = CARTPOLE_NL(X, u, P) with X 4-by-N (columns [x; xdot; q; qdot]) and u 1-by-N
%   (commanded cart acceleration, m/s^2), for the plant parameters of rshac_params:
%
%     d2x/dt2 = u
%     d2q/dt2 = (m g L sin(q) - m L cos(q) u - k qdot + tau) / (I + m L^2)
%
%   dX = CARTPOLE_NL(X, u, P, tau) adds an external torque tau (N m) on the pendulum,
%   used by the torque-pulse test E7 (Section 4.6). tau defaults to 0, which is Eq. (12).

if nargin < 4
    tau = 0;
end
den = P.m*P.L^2 + P.I;
q   = X(3, :);
dX  = [X(2, :);
       u;
       X(4, :);
       (P.m*P.g*P.L*sin(q) - P.m*P.L*cos(q).*u - P.k*X(4, :) + tau)/den];
end
