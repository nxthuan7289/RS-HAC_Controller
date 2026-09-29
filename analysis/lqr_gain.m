function K = lqr_gain()
%LQR_GAIN  LQR reference gain of Eqs. (24)-(25), convention u = +K*X, X = [x xd q qd].
%
%   Continuous-time LQR on the linearised model of Eq. (13) with
%       Q = diag(4, 0.1, 11, 0.2),   R = 0.02            (Eq. (24))
%   i.e. K = -lqr(A, B, Q, R). This is the gain computed by simulation/discrete_lqr.m
%   and used by the Simulink model for Tables 4-5. It is stored here to full precision
%   so that the scripts in experiments/ do not need the Control System Toolbox;
%   verify_gain_formula.m checks it against lqr().

K = [14.14213562 11.82904013 56.58791277 7.983345267];
end
