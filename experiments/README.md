# Robustness experiments E3–E7

These scripts implement five finite-horizon simulations on the nominal cart-pole plant in Eq. (12). They are separate from the Simulink runs because the Simulink model adds two small pendulum-joint friction terms to Eq. (12), -Fc*tanh(kt*qd) - kdr*qd^2 in the numerator of the pendulum equation (Fc = 4e-4 N m, kt = 150 s/rad, kdr = 1e-6 N m s^2/rad^2; see simulation/init_HAC_simulation.m). Do not compare these thresholds directly with the main simulation tables.

## Run

Use MATLAB R2022b with Fuzzy Logic Toolbox. Simulink is not required.

```matlab
cd experiments
run_experiments
```

Each script adds the package's `analysis/` folder to the MATLAB path and reads the fuzzy-controller definition from `simulation/fuzzy_ctrl_v2.fis`. The scripts save text and MAT results under `experiments/logs/`; rerunning a script replaces its corresponding log files.

| Script | Test |
|---|---|
| `exp_e3_noise.m` | Sensor noise and encoder quantization, with paired noise realizations across controllers. |
| `exp_e4_saturation.m` | Actuator bounds applied outside each fixed control law. |
| `exp_e5_delay.m` | Measurement delay on a 1 ms sampling grid. |
| `exp_e6_montecarlo.m` | Parameter uncertainty over 500 shared plant draws; controllers are not retuned. |
| `exp_e7_disturbance.m` | Recovery from a 50 ms torque pulse applied at the equilibrium. |

`run_experiments.m` runs all five in sequence. `exp_setup.m`, `simulate_fixed_ctrl.m`, `fc_law.m`, and `hac_weights.m` provide shared setup, integration, baseline, and weighting functions.

## Protocol and interpretation

The controller laws and de-semantization scale remain fixed. E4 applies actuator saturation as a separate plant-side limit. The plant is integrated by RK4 with ten substeps per 1 ms control period. Controller cases are selected per experiment: E3, E6, and E7 compare RS-HAC, LQR, and FC-clip; E4 and E5 also include FC-fis and FC-lin as out-of-range sensitivity cases. FC-clip models the fuzzy controller's declared semantic input range and is the FC reported in Table 7 of the paper; FC-fis and FC-lin are not reported in the paper. RS-HAC uses the table's linear end-segment extrapolation.

E3 uses a survival criterion (the pole remains upright and the cart remains on the track for the full horizon) and reports terminal RMS measures. The encoder and velocity-estimation chain make the ideal final band in Eq. (26) unsuitable as an E3 success test. E4–E7 require the track and pole constraints throughout the horizon and the true state to remain in the Eq. (26) band for the final second. E4, E5, and E7 are deterministic grid or bracket tests; their endpoints are finite-horizon pass/fail observations, not exact stability margins. The tests do not combine multiple stresses.

E6 uses 500 shared plant draws within ±30% of nominal for every controller. E3 and E6 use fixed random seeds. Archived run outputs are included in `logs/`. Their generic setup message predates the per-experiment controller selection, so use the result headings and scripts to identify the controllers compared.
