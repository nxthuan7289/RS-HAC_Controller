function M = simulate_fixed_ctrl(ctrlfun, X0, P, opt)
%SIMULATE_FIXED_CTRL  One Experiment-1 run of a FIXED controller with an independent
%   actuator bound, measurement delay, measurement chain and impulsive disturbance.
%   Shared by experiments E3, E4, E5, E6 and E7.
%
%   M = SIMULATE_FIXED_CTRL(ctrlfun, X0, P, opt)
%     ctrlfun  handle X (4-by-1 measured state) -> unsaturated command u [m/s^2]
%     X0       initial state [x; xdot; q; qdot]
%     P        rshac_params (plant, T_s). E6 perturbs the plant fields of P while the
%              controller keeps the nominal design it was built with.
%     opt      .uSat   actuator bound [m/s^2]. This is the SATURATION BLOCK ONLY.
%                      The de-semantization scale of Eq. (19) stays at P.u_max inside
%                      the control law, so sweeping uSat does not change the gain of
%                      the controller -- this keeps E4 as an actuator-limit study.
%              .dMs    pure delay on the whole measured state vector [ms], applied as
%                      dMs/1000/T_s samples (T_s = 1 ms, so integer ms are exact).
%              .T      horizon [s]                 (default 10)
%              .dwell  required time inside the band at the end of the run (default 1 s)
%              .meas   measurement chain, experiment E3. Absent or [] = the controller
%                      is fed the exact state, which is what E4, E5 and E6 use.
%                        .sigma_q  std of additive Gaussian noise on the MEASURED ANGLE
%                                  [rad], applied before the velocity estimator so that
%                                  the same noise is not counted twice
%                        .step_q   encoder quantum [rad], 0 for none
%                        .estimator true -> xdot and qdot are NOT measured but rebuilt
%                                  from x and q by the filtered derivative of the
%                                  firmware (state_derivatives.cpp): with T = 0.02 s,
%                                     v = (meas - z)/T,  z <- z + T_s v,  z(0) = meas(0)
%                        .seed     RNG seed, so that every controller sees the SAME
%                                  noise realization in a given repetition
%              .dist   impulsive disturbance, experiment E7. Absent or [] = none.
%                        .t0   start [s]   .dur  width [s]   .amp  torque [N m]
%
%   Same plant (Eq. 12, cartpole_nl), same 1 ms zero-order hold and 10 RK4 sub-steps
%   as simulate_exp1.m, which was checked against the published Simulink traces in
%   ../analysis/logs/script_vs_simulink.txt. Two protocol corrections are made here:
%     - the run is NOT stopped at the first entry into the band of Eq. (26); it runs
%       the full horizon, so control effort is compared over a common horizon and a
%       late escape or a limit cycle is caught;
%     - success requires the state to STAY inside the band for the last opt.dwell
%       seconds, which separates "first entry" from "settled".
%   Success is always judged on the TRUE state, never on the measured one.
%
%   Returned:
%     M.tEnter  first entry into the band of Eq. (26)      [s]  (NaN if never)
%     M.tSettle last exit from that band                   [s]  (0 if it never leaves)
%     M.Dx      max |x| over the run                       [m]
%     M.Dq      max |q| over the run                       [rad]
%     M.Su      sum |u_k| T_s over the FULL horizon, Eq. (27)
%     M.uPre    max |u| before saturation                  [m/s^2]
%     M.satFrac fraction of samples with |u| > uSat
%     M.uRms    RMS of the applied command over the last second of the horizon
%     M.qRms    RMS of the true angle over the last second of the horizon   [rad]
%     M.alive   survived the horizon: on the track and pole up, whatever the band did
%     M.success M.alive AND settled inside the band of Eq. (26) for the last dwell
%     M.why     failure reason, '' when successful

if ~isfield(opt, 'T'),     opt.T     = 10;  end
if ~isfield(opt, 'dwell'), opt.dwell = 1;   end
if ~isfield(opt, 'dMs'),   opt.dMs   = 0;   end
if ~isfield(opt, 'meas'),  opt.meas  = [];  end
if ~isfield(opt, 'dist'),  opt.dist  = [];  end

sub    = 10;  hs = P.Ts/sub;  nK = round(opt.T/P.Ts);
band   = [0.02; 0.02; deg2rad(0.5); deg2rad(0.5)];        % Eq. (26)
nDwell = round(opt.dwell/P.Ts);
dStep  = round(opt.dMs*1e-3/P.Ts);
tailK  = round(1/P.Ts);                                   % last second, for the RMS

meas = opt.meas;
useMeas = ~isempty(meas);
if useMeas
    if ~isfield(meas, 'sigma_q'),   meas.sigma_q   = 0;     end
    if ~isfield(meas, 'step_q'),    meas.step_q    = 0;     end
    if ~isfield(meas, 'estimator'), meas.estimator = true;  end
    if isfield(meas, 'seed') && ~isempty(meas.seed)
        rs = RandStream('mt19937ar', 'Seed', meas.seed);   % same stream for every
    else                                                   % controller of a repetition
        rs = RandStream.getGlobalStream();
    end
    Tf = 0.02;                                             % filter time constant, firmware
end

dist = opt.dist;
useDist = ~isempty(dist);

zx = 0;  zq = 0;                                           % estimator states, shared
X = X0(:);                                                 % with the nested measure()
Y = measure(X, 0);                                         % first measurement
buf = repmat(Y, 1, dStep + 1);                             % history, oldest first
M   = struct('tEnter',NaN, 'tSettle',NaN, 'Dx',0, 'Dq',0, 'Su',NaN, 'uPre',0, 'satFrac',0, ...
             'uRms',NaN, 'qRms',NaN, 'alive',false, 'success',false, 'why','', 'kEnd',0);
inBand = false(1, nK);
uLog = zeros(1, nK);  qLog = zeros(1, nK);
Su = 0;  nSat = 0;  k = 0;  ok = true;
while k < nK
    k  = k + 1;
    uu = ctrlfun(buf(:, 1));                              % delayed measurement
    u  = min(max(uu, -opt.uSat), opt.uSat);               % actuator bound, E4 variable
    M.uPre = max(M.uPre, abs(uu));
    nSat   = nSat + (abs(uu) > opt.uSat);
    Su     = Su + abs(u)*P.Ts;
    uLog(k) = u;

    tau = 0;                                              % E7 disturbance torque
    tk  = (k - 1)*P.Ts;
    if useDist && tk >= dist.t0 && tk < dist.t0 + dist.dur
        tau = dist.amp;
    end
    for s = 1:sub
        k1 = cartpole_nl(X, u, P, tau);            k2 = cartpole_nl(X + hs/2*k1, u, P, tau);
        k3 = cartpole_nl(X + hs/2*k2, u, P, tau);  k4 = cartpole_nl(X + hs*k3, u, P, tau);
        X  = X + hs/6*(k1 + 2*k2 + 2*k3 + k4);
    end
    buf = [buf(:, 2:end), measure(X, k)];
    M.Dx      = max(M.Dx, abs(X(1)));
    M.Dq      = max(M.Dq, abs(X(3)));
    qLog(k)   = X(3);
    inBand(k) = all(abs(X) <= band);
    if abs(X(1)) > 0.43,  ok = false;  M.why = 'track limit |x| > 0.43 m';  break, end
    if abs(X(3)) >= pi/2, ok = false;  M.why = 'pole fell |q| >= 90 deg';   break, end
    if ~isfinite(X(1)),   ok = false;  M.why = 'diverged';                  break, end
end
M.kEnd    = k;
M.alive   = ok;
M.satFrac = nSat/k;
ie = find(inBand(1:k), 1);
if ~isempty(ie), M.tEnter = ie*P.Ts; end
if ok
    M.Su   = Su;
    last   = find(~inBand(1:k), 1, 'last');
    if isempty(last), last = 0; end
    M.tSettle = last*P.Ts;
    if k >= tailK
        M.uRms = sqrt(mean(uLog(k-tailK+1:k).^2));
        M.qRms = sqrt(mean(qLog(k-tailK+1:k).^2));
    end
    if ~isempty(ie) && (k - last) >= nDwell
        M.success = true;
    else
        M.why = sprintf('not inside Eq.(26) for the last %g s', opt.dwell);
    end
end

% ---------------------------------------------------------------------------------
    function Y = measure(Xt, kk)
    %MEASURE  Sensor chain of experiment E3. Without opt.meas the controller is fed
    %   the exact state, which is what E4, E5 and E6 use, bit for bit as before.
        if ~useMeas
            Y = Xt(:);
            return
        end
        qm = Xt(3);
        if meas.sigma_q > 0, qm = qm + meas.sigma_q*randn(rs); end
        if meas.step_q  > 0, qm = round(qm/meas.step_q)*meas.step_q; end
        xm = Xt(1);
        if kk == 0, zx = xm;  zq = qm; end                % IC loading of the estimator
        if meas.estimator
            vx = (xm - zx)/Tf;   zx = zx + P.Ts*vx;       % filtered derivative,
            vq = (qm - zq)/Tf;   zq = zq + P.Ts*vq;       % state_derivatives.cpp
        else
            vx = Xt(2);  vq = Xt(4);
        end
        Y = [xm; vx; qm; vq];
    end
end
