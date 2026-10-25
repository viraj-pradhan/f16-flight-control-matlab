# F-16 Flight Dynamics and Control (MATLAB)

Nonlinear 6-DOF flight model of an F-16-style fighter, with trim, linearization and an LQR stability augmentation system (SAS), built in MATLAB.

The F-16 is designed with relaxed static stability: very agile, but the bare airframe is unstable and needs a flight control computer to fly. This project reproduces that effect and then fixes it with feedback control.

## Results

**Step 1: Open-loop response.** The same elevator doublet applied to an F-16-like unstable airframe and to a statically stable one.

![Open-loop response](f16_step1_openloop.png)

**Step 2: Pitch SAS.** The unstable airframe with the SAS off and on.

![SAS off vs on](f16_step2_sas.png)

### Key numbers (500 ft/s, sea level, level flight)

| Quantity | Value |
|---|---|
| Trim angle of attack | 2.20 deg |
| Trim throttle / elevator | 0.102 / 0.39 deg |
| Trim residual | ~1e-11 |
| Unstable short-period pole (SAS off) | +0.458 /s (doubling time about 1.5 s) |
| Short-period poles (SAS on) | -3.87 ± 2.50i (damping about 0.84) |
| LQR gains | K_w = -0.0028 rad/(ft/s), K_q = -0.554 rad/(rad/s) |
| Peak elevator during doublet (SAS on) | about 3.5 deg |

### Open-loop modes (F-16-like case)

| Mode | Poles |
|---|---|
| Short period | -2.34 and **+0.458** (unstable) |
| Phugoid | -0.020 ± 0.116i (period about 50 s) |
| Dutch roll | -0.321 ± 2.429i (damping about 0.13) |
| Roll | -3.43 |
| Spiral | -0.018 |

## What is implemented

- **Nonlinear 6-DOF model** in body axes with 13 states: body velocities (u, v, w), Euler angles, body rates, position, altitude and engine power. It includes an ISA atmosphere, wind-to-body force transformation, inertia coupling (Ixz) and a first-order engine lag.
- **Trim solver** for steady level flight (`fminsearch` over angle of attack, throttle and elevator).
- **Numerical linearization** (central differences) and mode analysis.
- **Pitch SAS** using LQR on the short-period subsystem, feeding back w (about angle of attack) and pitch rate q to the elevator, then tested on the full nonlinear model.

## How to run

Requirements:
- MATLAB R2020a or newer (uses `exportgraphics`)
- Control System Toolbox (`lqr`, `damp`)

Steps:
1. Put all files in one folder and make it MATLAB's current folder.
2. Run `run_f16_step1` for trim, modes and the open-loop doublet simulation.
3. Run `run_f16_step2_sas` for the SAS design and the SAS off vs on comparison.

Each script prints its results in the Command Window and saves a PNG in the current folder.

The `run_` files are the entry points. The other files are functions called by them and are not meant to be run directly.

## Files

| File | Purpose |
|---|---|
| `run_f16_step1.m` | Trim, linearization, open-loop simulation and plot |
| `run_f16_step2_sas.m` | LQR pitch SAS design and comparison plot |
| `f16_dynamics.m` | Nonlinear 6-DOF equations of motion |
| `f16_trim.m` | Steady level-flight trim solver |
| `f16_params.m` | Aircraft constants and aerodynamic coefficients |

## Conventions

- Units: ft, slug, lb, rad, s
- Body axes, flat earth
- Elevator: positive = trailing edge down = nose down
- Control vector: `[throttle (0 to 1); elevator; aileron; rudder]`, surfaces in radians
- State vector: `[u v w phi theta psi p q r pn pe h pow]`

## Limitations

- Mass, inertia and geometry follow the Stevens & Lewis F-16, but the **aerodynamic coefficients are simplified linear values**, not the NASA TP-1538 lookup tables. The model shows realistic qualitative behavior (relaxed stability, unstable short period) but is not validated against real F-16 data.
- There is no stall model, so results are only meaningful at moderate angles of attack. Keep simulations short for the unstable case.
- No actuator dynamics, sensor noise or gusts yet.

## Roadmap

- [x] Nonlinear model, trim and linearization
- [x] Pitch SAS (LQR)
- [ ] Yaw damper and roll damper
- [ ] Autopilot: pitch attitude, altitude and heading hold
- [ ] PID vs LQR comparison
- [ ] Gain scheduling across the flight envelope
- [ ] Actuator dynamics, sensor noise and gusts
- [ ] NASA TP-1538 aerodynamic tables
- [ ] 3D flight playback

## References

- B. L. Stevens and F. L. Lewis, *Aircraft Control and Simulation*, Wiley.
- L. T. Nguyen et al., *Simulator Study of Stall/Post-Stall Characteristics of a Fighter Airplane With Relaxed Longitudinal Static Stability*, NASA TP-1538, 1979.

## Author

Viraj Pradhan
LinkedIn: www.linkedin.com/in/virajdpradhan

## License

MIT, see [LICENSE](LICENSE).
