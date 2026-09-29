function P = rshac_params()
%RSHAC_PARAMS  Single source of truth for the cart-pole plant and the RS-HAC design.
%
%   Values are taken from the paper so that every analysis script
%   in this folder is provably analysing the controller that was actually run:
%     - plant           : Section 4.1
%     - semantization   : Eqs. (15)-(18)
%     - SQSM parameters : Table 3
%     - de-semantization: Eq. (19)
%     - weights         : Eqs. (20)-(22)
%
%   Cross-checked against simulation/init_HAC_simulation.m,
%   simulation/my_HAC_improved.m, and the firmware under firmware/.

%% ---- plant (Section 4.1) -------------------------------------------------
P.m  = 0.116527;          % pendulum mass                       [kg]
P.L  = 0.15;              % hinge to centre of gravity          [m]
P.g  = 9.80665;           % gravity                             [m/s^2]
P.I  = (1/3)*P.m*P.L^2;   % inertia about the CoG               [kg m^2]  (= 8.7395e-4)
P.k  = 0.000161;          % revolute joint damping              [N s/rad]
P.Ts = 0.001;             % sampling period                     [s]

den  = P.m*P.L^2 + P.I;
P.A  = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 P.m*P.g*P.L/den -P.k/den];   % Eq. (13)
P.B  = [0; 1; 0; -P.m*P.L/den];

%% ---- de-semantization (Eq. 19) ------------------------------------------
P.u_max =  3*P.g;         % 29.41995 m/s^2
P.u_min = -3*P.g;

%% ---- adaptive weights (Eqs. 20-22) --------------------------------------
P.l1 = deg2rad(5);        % 0.09 rad in the paper (rounded); 5 deg = 0.0873 rad
P.l2 = deg2rad(50);       % 0.87 rad in the paper (rounded); 50 deg = 0.8727 rad

%% ---- implementation conventions -----------------------------------------
% oor      : rule of the SQSM lookup outside [v(1), v(n)]
%            'extrap'  linear extrapolation of the end segments -- what is implemented
%                      (interp1 'extrap' in my_HAC_improved.m, look1_binlx in the firmware)
%            'clip'    hold the end values
%            'haend'   append the hedge-algebra constants 0 -> 0 and 1 -> 1 (C = {0, W, 1})
% saturate : explicit output saturation |u| <= u_max when enabled. The
%            Simulink model has no output saturation on its live path.
%            Off for the sector / stability analysis.
P.oor      = 'extrap';
P.saturate = false;

%% ---- per-channel design (Table 3) ---------------------------------------
% ch.n      : number of linguistic labels  n_<=
% ch.a_i    : alpha of the STATE           (Table 3, row alpha, state column)
% ch.a_u    : alpha of the INTERMEDIATE CONTROLLER
% ch.th_i   : theta of the state,  ch.th_u : theta of the controller
% ch.sem    : 'lin' (Eqs. 15-16) or 'igs' (Eqs. 17-18)
% ch.par    : [min max] for 'lin', or the slope a for 'igs' (c = 0 for symmetry)
P.chan = {
  struct('name','x'  ,'n',7,'a_i',0.5,'a_u',0.350,'th_i',0.5,'th_u',0.5,'sem','lin','par',[-0.43 0.43])
  struct('name','xd' ,'n',5,'a_i',0.5,'a_u',0.800,'th_i',0.5,'th_u',0.5,'sem','lin','par',[-2.00 2.00])
  struct('name','q'  ,'n',5,'a_i',0.5,'a_u',0.725,'th_i',0.5,'th_u',0.5,'sem','igs','par',8.00)
  struct('name','qd' ,'n',7,'a_i',0.5,'a_u',0.800,'th_i',0.5,'th_u',0.5,'sem','igs','par',0.45)
};

%% ---- balance-mode operating envelope ------------------------------------
% RS-HAC is only in the loop for |q| <= 15 deg (swing-up handover, Section 4.5.1);
% |x| is bounded by the physical track, the velocity bounds are the measured maxima.
P.env = struct('x',0.43, 'xd',2.0, 'q',deg2rad(15), 'qd',deg2rad(200));
end
