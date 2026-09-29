function run_experiments()
%RUN_EXPERIMENTS  Run the robustness experiments of this folder, in order.
%
%   >> cd experiments
%   >> run_experiments
%
%   Runs, with no Simulink licence and in about half an hour on a desktop machine:
%     exp_e3_noise        sensor noise and encoder quantization, 100 repetitions per level
%     exp_e4_saturation   actuator-bound sweep at a fixed control law
%     exp_e5_delay        measurement-delay sweep at a fixed control law
%     exp_e6_montecarlo   parameter uncertainty, 500 plant draws
%     exp_e7_disturbance  largest recoverable impulsive disturbance
%
%   The deterministic sweeps (E4, E5, E7) take about a minute each; E3 and E6 are the
%   long ones because they repeat a 10 s run a few hundred times per controller.
%
%   Every script re-verifies its controllers against the published small-signal gains
%   before it sweeps anything, and writes its full sweep, its refinement steps and the
%   failure reason of every run to logs/. Nothing here needs the Simulink model: the
%   plant is the nominal cart-pole of Eq. (12) integrated by RK4, and the control laws
%   are the explicit ones in ../analysis.
%
%   E3 and E6 use a fixed seed and the SAME realizations for every controller, so the
%   comparisons are paired and reproducible; both print the paired differences.
%
%   See README.md in this folder for the protocol, the results and their limits.

exp_e3_noise
exp_e4_saturation
exp_e5_delay
exp_e6_montecarlo
exp_e7_disturbance
end
