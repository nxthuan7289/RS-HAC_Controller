%G1_OUT_OF_RANGE_QUICKLOOK  Illustrative effect of the out-of-range lookup rule.
%
%   Script-level simulation of RS-HAC with the three candidate rules for the SQSM
%   lookup outside its table -- 'extrap' (as implemented), 'clip', 'haend' (append
%   the hedge-algebra constants 0 -> 0 and 1 -> 1):
%     plant       nonlinear cart-pole, Eq. (12) (cartpole_nl.m)
%     controller  rshac_law_vec, sampled at T_s = 1 ms with zero-order hold, explicit
%                 saturation |u| <= u_max; balance controller only (no swing-up switch)
%     scenarios   Experiment 1: X0 = [0 0 q0 0], q0 = 10, 20, 30 deg
%     indices     Delta_t   first time Eq. (26) holds -- the Simulink runs stop there
%                           (get_goal_time.m uses the last time stamp of the cut data)
%                 Delta_x_m max |x| up to Delta_t
%                 Sigma_u   sum |u_k| T_s up to Delta_t, Eq. (27)
%
%   This script-level result is illustrative and is not a replacement for the
%   Simulink results. The model and controller conventions differ as described
%   in the experiment logs.
%
%   Run:  >> g1_out_of_range_quicklook        Log: logs/g1_out_of_range_quicklook.txt

clear; clc;
here   = fileparts(mfilename('fullpath'));
logDir = fullfile(here, 'logs');
if ~exist(logDir, 'dir'), mkdir(logDir); end
logFile = fullfile(logDir, 'g1_out_of_range_quicklook.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
fprintf('g1_out_of_range_quicklook   %s   MATLAB %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version);

P0 = rshac_params();
P0.saturate = true;
rules  = {'extrap', 'clip', 'haend'};
q0s    = [10 20 30];
table4 = [2.052 0.106 1.194; 2.169 0.187 2.835; 2.598 0.306 5.121];     % RS-HAC, Table 4
T      = 10;
sub    = 10;
hs     = P0.Ts/sub;
band   = [0.02; 0.02; deg2rad(0.5); deg2rad(0.5)];                     % Eq. (26)

fprintf('%-7s %5s %9s %10s %10s %13s %8s %8s\n', 'rule', 'q0', 'Dt [s]', 'Dx_m [m]', 'Su [m/s]', 'max|u| unsat', 'sat [%]', 'success');
R = struct([]);
for r = 1:numel(rules)
    P   = P0;  P.oor = rules{r};
    Pns = P;   Pns.saturate = false;
    for j = 1:numel(q0s)
        X = [0; 0; deg2rad(q0s(j)); 0];
        tHit = NaN;  Su = 0;  Dx = 0;  uPre = 0;  nSat = 0;  ok = true;  k = 0;
        while k < round(T/P.Ts)
            k   = k + 1;
            u   = rshac_law_vec(X, P);
            uu  = rshac_law_vec(X, Pns);
            uPre = max(uPre, abs(uu));
            nSat = nSat + (abs(uu) > P.u_max);
            Su   = Su + abs(u)*P.Ts;
            for s = 1:sub
                k1 = cartpole_nl(X, u, P);              k2 = cartpole_nl(X + hs/2*k1, u, P);
                k3 = cartpole_nl(X + hs/2*k2, u, P);    k4 = cartpole_nl(X + hs*k3, u, P);
                X  = X + hs/6*(k1 + 2*k2 + 2*k3 + k4);
            end
            Dx = max(Dx, abs(X(1)));
            if abs(X(1)) > 0.43 || abs(X(3)) > pi/2
                ok = false;
                break
            end
            if all(abs(X) <= band)
                tHit = k*P.Ts;
                break
            end
        end
        success = ok && ~isnan(tHit);
        fprintf('%-7s %5d %9.3f %10.3f %10.3f %13.2f %8.1f %8d\n', rules{r}, q0s(j), tHit, Dx, Su, uPre, 100*nSat/k, success);
        R(r, j).rule = rules{r};  R(r, j).q0 = q0s(j);  R(r, j).Dt = tHit;  R(r, j).Dx = Dx;
        R(r, j).Su = Su;  R(r, j).uPre = uPre;  R(r, j).satFrac = nSat/k;  R(r, j).success = success;
    end
end
fprintf('\npublished Table 4, RS-HAC (Simulink):\n');
for j = 1:numel(q0s)
    fprintf('%-7s %5d %9.3f %10.3f %10.3f\n', 'Table 4', q0s(j), table4(j, 1), table4(j, 2), table4(j, 3));
end

save(fullfile(logDir, 'g1_out_of_range_quicklook.mat'), 'R');
fprintf('\nSaved logs/g1_out_of_range_quicklook.mat\n');
diary off;
