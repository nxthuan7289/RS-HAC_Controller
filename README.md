# RS-HAC cart-pole code and data

This repository contains the MATLAB/Simulink code, Teensy firmware, and saved data associated with the paper *A Recursive Semantic Hedge Algebra Controller for Cart-Pole Stabilization with Real-Time Experimental Validation*.

## Repository layout

| Folder | Contents |
|---|---|
| `analysis/` | Semantic maps, controller equations, stability and theory calculations, and their logs. |
| `simulation/` | Simulink model, controller baselines, and scripts for the reported simulation cases. |
| `experiments/` | Robustness scripts of Section 4.6 (E3–E7) and saved logs. |
| `data/` | Saved simulation runs and the hardware trace used by the paper. |
| `firmware/` | Teensy 4.1 application, controller code, and code-generation models. |

## Where each result of the paper comes from

| Paper | Source in this repository |
|---|---|
| Tables 1–2, Propositions 1–2, Lemma 1 | `analysis/check_sqsm_properties.m` → `analysis/logs/check_sqsm_properties.txt` |
| Section 3.6: K_RS, closed-loop eigenvalues, Corollary 1 example | `analysis/verify_gain_formula.m` → `analysis/logs/verify_gain_formula.txt`; `analysis/inverse_design_examples.m` |
| Table 3, Eqs. (15)–(22) | `analysis/rshac_params.m` (also `simulation/init_HAC_simulation.m` and the firmware) |
| Tables 4–5, Figures 8–9 | `simulation/cartPend_HAC.slx` with `simulation/init_HAC_simulation.m`; saved runs in `data/*_experiment_*.mat`; metrics by `simulation/get_goal_time.m` → `data/goal_time_RSHAC.xlsx` |
| Figure 11 | `data/RSHAC_real_swing_up.csv` and `data/RSHAC_real_condition.mat`, plotted by `simulation/RSHAC_real_data_processing.m` |
| Table 6 (execution time) | `firmware/src/main.cpp` (timing output, see below) |
| Table 7 | `experiments/run_experiments.m` → `experiments/logs/` |

## Requirements

- MATLAB R2022b.
- For the Simulink comparisons: Simulink, Control System Toolbox, and Fuzzy Logic Toolbox.
- For the LMI and margin analyses: Robust Control Toolbox/LMI Lab and Control System Toolbox.
- For the standalone E3–E7 scripts: MATLAB R2022b and Fuzzy Logic Toolbox; Simulink is not required.
- For firmware builds: PlatformIO with the Teensy 4.1 platform and the libraries listed in `firmware/platformio.ini`.

## Reproduction entry points

### Theory and controller calculations

From MATLAB, change to `analysis/` and run the scripts in this order:

1. `check_sqsm_properties.m`
2. `verify_gain_formula.m`
3. `rshac_stability.m`
4. `small_signal_margins.m`
5. `rshac_lmi_certificate.m`
6. `validate_invariant_set.m`
7. `inverse_design_examples.m`
8. `g1_out_of_range_quicklook.m`
9. `make_theory_figures.m`

The figure script uses the certificate files in `analysis/` and writes its output to `figures/` at the package root.

The stability result of the paper is the local one of Proposition 3 (Level 1 of `rshac_stability.m`). The quadratic certificates of `rshac_stability.m` (Level 2), `rshac_lmi_certificate.m` and `validate_invariant_set.m` are supplementary calculations and are not reported in the paper.

### Simulink results

Open `simulation/init_HAC_simulation.m`, set the initial condition, controller case, and output options for the desired run, then execute it from `simulation/`. The script defaults to `saveData = false`. The Simulink model is `simulation/cartPend_HAC.slx`.

### Robustness experiments

From MATLAB, change to `experiments/` and run:

```matlab
run_experiments
```

This runs E3–E7 and writes text and MAT logs under `experiments/logs/`. Individual scripts can be run separately. The experiments use the nominal plant of Eq. (12) with RK4 integration. That plant omits the friction terms of the Simulink model: the Simulink model adds two small pendulum-joint friction terms to Eq. (12), -Fc*tanh(kt*qd) - kdr*qd^2 in the numerator of the pendulum equation (Fc = 4e-4 N m, kt = 150 s/rad, kdr = 1e-6 N m s^2/rad^2; see simulation/init_HAC_simulation.m). E3–E7 results should therefore be interpreted separately from the Simulink results of Tables 4–5.

The fuzzy controller of the paper is the `FC-clip` case (semantic inputs clipped to [0.25, 0.75], Section 4.3). E4 and E5 also report `FC-lin` and `FC-fis` as sensitivity cases for inputs outside that range; these are not reported in the paper (see `experiments/fc_law.m`).

### Firmware

Open `firmware/` as a PlatformIO project. Build with `pio run`; the target is Teensy 4.1.

The firmware reports DWT cycle-counter timing for `controller_oneStep()` every 1,000 control steps as a line `TIMING us: min=… mean=… max=… n=1000`; serial output is outside the timed region. `controller_oneStep()` runs the generated controller step, which contains RS-HAC together with the swing-up switching logic.

`firmware/generated-original/` holds the code generated from `firmware/model/CODE_GEN.slx`. The copy in `firmware/lib/HAC/` differs in one line: the switching angle between the energy swing-up controller and RS-HAC was changed by hand from the generated 25° to 15°, the value used in the paper (Section 4.5.1).

## Scope of the evidence

The analytical stability result is local to the nominal continuous-time balance model. It does not prove stability of the sampled firmware, swing-up switching, or every state in the operating envelope. The robustness scripts test finite horizons and specified disturbance cases; their passing and failing grid points are not global stability boundaries. E3 reports survival and terminal RMS measures, while E4–E7 use the stated final-band criterion.

The paper reports results for the listed model, controller parameters, and baselines. The results do not establish a tuning-independent ranking against other fuzzy or nonlinear controllers.

## Citation and license

Paper title: *A Recursive Semantic Hedge Algebra Controller for Cart-Pole Stabilization with Real-Time Experimental Validation*.

Complete citation metadata will be added when the publication details are final.
