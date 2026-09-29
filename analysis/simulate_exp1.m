function M = simulate_exp1(ctrl, X0, P, T)
%SIMULATE_EXP1  One Experiment-1 run on the nonlinear plant with a sampled, saturated controller.
%
%   M = SIMULATE_EXP1(ctrl, X0, P, T)
%     ctrl  handle X (4-by-1) -> unsaturated command u (m/s^2)
%     X0    initial state [x; xdot; q; qdot]
%     P     rshac_params (plant, T_s, u_max)
%     T     horizon in seconds
%
%   The command is computed every T_s = 1 ms, saturated to |u| <= u_max and held
%   (zero-order hold) over 10 RK4 sub-steps of the nonlinear plant, Eq. (12).
%   As in the Simulink runs (get_goal_time.m), the run stops when the band of
%   Eq. (26) is first reached. Returned indices:
%     M.Dt       first time Eq. (26) holds (NaN if never)
%     M.Dx       max |x| up to Dt
%     M.Su       sum |u_k| T_s up to Dt, Eq. (27)
%     M.uPre     max |u| before saturation;  M.satFrac  fraction of saturated samples
%     M.success  Dt reached, |x| <= 0.43 m and |q| < 90 deg throughout
%     M.t, M.X, M.u  traces at the controller rate

sub  = 10;
hs   = P.Ts/sub;
band = [0.02; 0.02; deg2rad(0.5); deg2rad(0.5)];
nK   = round(T/P.Ts);

X = X0(:);
M = struct('Dt', NaN, 'Dx', 0, 'Su', 0, 'uPre', 0, 'satFrac', 0, 'success', false, ...
           't', zeros(1, nK), 'X', zeros(4, nK), 'u', zeros(1, nK));
nSat = 0;
ok   = true;
k    = 0;
while k < nK
    k  = k + 1;
    uu = ctrl(X);
    u  = min(max(uu, P.u_min), P.u_max);
    M.uPre = max(M.uPre, abs(uu));
    nSat   = nSat + (abs(uu) > P.u_max);
    M.Su   = M.Su + abs(u)*P.Ts;
    for s = 1:sub
        k1 = cartpole_nl(X, u, P);              k2 = cartpole_nl(X + hs/2*k1, u, P);
        k3 = cartpole_nl(X + hs/2*k2, u, P);    k4 = cartpole_nl(X + hs*k3, u, P);
        X  = X + hs/6*(k1 + 2*k2 + 2*k3 + k4);
    end
    M.t(k) = k*P.Ts;  M.X(:, k) = X;  M.u(k) = u;
    M.Dx = max(M.Dx, abs(X(1)));
    if abs(X(1)) > 0.43 || abs(X(3)) >= pi/2
        ok = false;
        break
    end
    if all(abs(X) <= band)
        M.Dt = k*P.Ts;
        break
    end
end
M.t = M.t(1:k);  M.X = M.X(:, 1:k);  M.u = M.u(1:k);
M.satFrac = nSat/k;
M.success = ok && ~isnan(M.Dt);
end
