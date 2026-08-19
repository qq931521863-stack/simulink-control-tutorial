# Changelog

All notable changes to this project are documented here.

## [2.1.0] - 2026-08-20

### Added
- 13 new tutorials in Phase 7 (Nonlinear Filtering) and Phase 8 (Simulink Advanced):
  - t31: EKF basics — Van der Pol oscillator state estimation with Extended Kalman Filter
  - t32: EKF pendulum — inverted pendulum state estimation, strong nonlinearity
  - t33: UKF basics — Unscented Kalman Filter, sigma-point approach (no Jacobian needed)
  - t34: EKF vs UKF — Monte Carlo comparison across nonlinearity strengths
  - t35: IMU fusion — gyroscope + accelerometer attitude estimation
  - t36: Stateflow — state machine, motor start/stop control
  - t37: Triggered/Enabled Subsystem — conditional execution
  - t38: Bus signals — signal bundling and extraction
  - t39: Model Referencing — large-model decomposition
  - t40: Variant Subsystem — multiple control schemes in one model
  - t41: Model Callbacks — automatic init/save
  - t42: S-Function — custom blocks (dead-zone)
  - t43: Model Advisor — static analysis
- All 13 tutorials are standalone (Simulink model + MATLAB post-processing)
- Total tutorials: 30 → 43
- Comprehensive learning path: Kalman → EKF → UKF → sensor fusion → Stateflow → advanced Simulink

### Changed
- `run_all_tutorials.m`: added t31-t43 to batch run
- README badge updated: 30 → 43 lessons

### Fixed
- t21: sliding mode control u_eq sign (was reversed, caused divergence)
- t23: rewritten as standard MRAC (same pole, different DC gain); theta* corrected
- t27/t28: State-Space B matrix, ode45 step input, frequency units (Hz vs rad/s)
- t29/t30: LQR velocity feedback wiring, suspension damper added
- t31-t34: EKF F matrix (discrete I+dt*A), UKF alpha parameter
- t22/t25/t26: lead compensator pole-zero cancellation, honest rename, DH table
- t24/t36-t43: Simulink wiring/API fixes (fuzzy PI, Stateflow Chart, bus signals, etc.)

## [2.0.0] - 2026-07-18

## [1.3.0] - 2026-07-18

### Added
- 4 new tutorials: t22 (Root Locus), t23 (MRAC), t24 (Fuzzy), t25 (Simscape)
- Simulink models for t15-t21 (7 previously analysis-only scripts now have .slx files)
- `check_setup.m` — environment checker for beginners
- FAQ section in README (中文/English)
- Beginner-friendly quickstart guide with step-by-step instructions

### Changed
- Total tutorials: 21 → 25 (all 25 now have corresponding .slx models)
- Restructured course into 5 progressive Phases (was 3)
  - Phase 1: Simulink Basics → Phase 2: Classical Control → Phase 3: Modern Control
  - Phase 4: Advanced Control → Phase 5: Applications & Deployment
- Reorganized run_all_tutorials.m and t00_main_guide.m to follow learning path
- Updated README (中文/English) with new Phase tables, learning path diagram, and FAQ

### Fixed
- t23: Completed MIT adaptive law wiring (was marked TODO)
- t13: Wired Sum_FF feedforward path
- t12: Removed unused SumK_Luenberger block
- t24: Fixed Fuzzy Logic Toolbox fallback when toolbox not installed
- t25: Updated Simscape block paths for R2025a compatibility
- t21/t17/t24: Fixed dual-controller Plant block conflicts
- All save_system paths now use absolute paths (t22-t25)

## [1.2.0] - 2026-07-18

### Added
- Star History chart in README (中文/English)
- GitHub Actions CI workflow: syntax check, structure validation, monthly schedule
- CI passing badge in README

### Changed
- Updated all simulation images and models from latest run

## [1.1.0] - 2026-07-17

### Added
- 6 advanced control tutorials: t16 (frequency domain), t17 (lead-lag), t18 (system ID), t19 (H∞ robust), t20 (MPC), t21 (sliding mode)
- GitHub community health files: issue templates (bug, feature, tutorial request), PR template
- CODE_OF_CONDUCT.md and SECURITY.md
- FUNDING.yml

### Changed
- Total tutorials: 15 → 21
- README overhaul: added table of contents, quick start, prerequisites, collapsible sections, improved badges
- README_EN.md synced with latest structure
- Updated CONTRIBUTING.md with detailed guidelines

### Fixed
- t19 LQR state feedback closed-loop formula (line 138): replaced incorrect `feedback()` with proper `A-BK` formulation
- t19 fig3 layout: changed from 1×2 to clean 2×1, removed empty axes

## [1.0.0] - 2026-07-17

### Added
- 15 Simulink tutorials covering fundamentals, control theory, and mechatronic systems
- 14 `.slx` Simulink model files
- `run_all_tutorials.m` — one-click batch runner
- Simulation result figures in `docs/images/`
- English README (`README_EN.md`)
- Contributing guidelines (`CONTRIBUTING.md`)
- MIT License

### Tutorials
- **Phase 1 (t01-t08):** Signal basics, math blocks, first/second-order systems, PID control, sources/sinks, subsystems, masking
- **Phase 2 (t09-t12):** State-space models, LQR optimal control, Luenberger observer, Kalman filter
- **Phase 3 (t13-t15):** DC motor modeling, PMSM FOC vector control, embedded C code generation
