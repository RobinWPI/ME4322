# Homework 1 MATLAB Analysis

Final release: September 11, 2026.

## Run

Set MATLAB's Current Folder here and run:

```matlab
main
data_comparison_analysis
```

- `main.m` performs MATLAB-only mechanics calculations and opens five figures. It requires Symbolic Math Toolbox.
- `data_comparison_analysis.m` loads the saved MATLAB results, compares them with hard-coded native PMKS samples, and opens seven report figures. It does not run PMKS, access a browser, or read external CSV files.
- Both scripts must remain together. Outputs are written to `results_matlab/` and `results_pmks_comparison/`.

## Fixed model

6061-T6, supplied SOLIDWORKS masses and centroidal inertias, and the established A-H geometry. A, D and G are grounded. H is 1.843 m beyond F along GF. AB rotates counterclockwise at 23.1481481481481 RPM with zero input angular acceleration: one revolution per part, 12,500 parts in nine hours.

Both static and dynamic cases include link self-weight and the same global force **Q = (0, -200) N at H**. No additional artifact mass or inertia is inferred from Q. This replaces the older point-mass payload version. Initial MATLAB torques are -504.372530 N m static and -47.174905 N m dynamic.

MATLAB uses g = 9.81 m/s² and the unrounded mapped CAD centers. PMKS uses g = 9.80665 m/s² and saved 0.001 m coordinates. CAD screenshot inputs themselves have limited displayed precision.

## Comparison and figures

The September 11 native export set supplies **95 first-position values**, **39 embedded full-cycle series**, and **67 component comparisons**. All five links, A-H point velocities/accelerations, and all five COM accelerations are included. Each exported series contains 361 samples over 2.592 s; MATLAB uses 721 positions. Values are compared at matching times without phase fitting or amplitude scaling.

Seven report plots show all link angular velocities, all angular accelerations, H velocity, H acceleration, selected COM acceleration magnitudes, static torque comparison, and MATLAB static/dynamic torque. Axes carry units and every curve is identified. Solid lines are MATLAB; dashed lines and open markers are PMKS.

Newton's second-law forces are MATLAB-only in the report, following the checklist. The separate native dynamic-torque CSV is retained as a diagnostic, not embedded as report comparison data. Some full-cycle component differences exceed 1%; the saved error metrics report them directly. Zero-reference curves have undefined percentage error.

Geometry/input mismatches disable all references. Changed COMs disable COM/force references; changed masses, gravity or Q disable static-force references. Rerun `main` after changing the model and obtain new PMKS exports for a new comparison.

## Verification

```matlab
test_homework1_classroom
test_homework1_compare_pmks
test_homework1_standalone
```

Checks cover closure, force balance, energy rate, finite differences, complete reference coverage, and isolated execution of the two scripts. The standalone test retains a temporary test folder. Motor sizing, fatigue, startup, dwell, and pickup/release timing are outside this steady loaded-cycle calculation.
