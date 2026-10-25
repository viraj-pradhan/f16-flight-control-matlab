# F-16 Flight Dynamics and Control (MATLAB)

Nonlinear 6-DOF model of an F-16-style fighter with trim, linearization and an LQR stability augmentation system (SAS).

## Results

![Open-loop response](f16_step1_openloop.png)
![SAS off vs on](f16_step2_sas.png)

- Bare airframe has an unstable short-period pole at +0.46 /s (doubling time about 1.5 s)
- LQR pitch SAS moves the short-period poles to -3.9 ± 2.5i (damping about 0.84)

## How to run

Requirements: MATLAB with Control System Toolbox.

1. Put all files in one folder and make it MATLAB's current folder
2. Run `run_f16_step1` (trim, modes, open-loop doublet)
3. Run `run_f16_step2_sas` (pitch SAS, SAS off vs on)

## Files

| File | Purpose |
|---|---|
| `f16_params.m` | Aircraft constants and aerodynamic coefficients |
| `f16_dynamics.m` | Nonlinear 6-DOF equations of motion |
| `f16_trim.m` | Steady level-flight trim solver |
| `run_f16_step1.m` | Trim, linearization, open-loop simulation |
| `run_f16_step2_sas.m` | LQR pitch SAS design and comparison |

## Notes

Mass, inertia and geometry follow the Stevens & Lewis F-16. Aerodynamic coefficients are simplified linear values, not the NASA TP-1538 tables.

## Roadmap

- [x] Nonlinear model, trim, linearization
- [x] Pitch SAS (LQR)
- [ ] Yaw and roll dampers
- [ ] Autopilot (pitch attitude, altitude, heading)
- [ ] PID vs LQR comparison
- [ ] NASA TP-1538 aerodynamic tables
- [ ] 3D flight playback

Author: [Your name]
