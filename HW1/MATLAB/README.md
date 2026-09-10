# Homework 1: MATLAB calculation and PMKS comparison

## Run order

Set MATLAB's Current Folder to this directory, then run:

```matlab
main
data_comparison_analysis
```

The two scripts have separate purposes:

| Script | Purpose | Output folder |
| --- | --- | --- |
| `main.m` | MATLAB mechanics calculations, MATLAB-only tables and five figures | `results_matlab` |
| `data_comparison_analysis.m` | Compare saved MATLAB results with hard-coded PMKS reference data | `results_pmks_comparison` |

Run only the first script when you need the MATLAB analysis. Run the second when you need comparison tables, error metrics and the four-panel comparison figure. The comparison script loads `results_matlab/classroom_results.mat`; it does not rerun the mechanics calculations. After changing geometry, input speed, material or loading, rerun the main script before running the comparison.

The main script requires MATLAB with Symbolic Math Toolbox. The comparison script performs no symbolic computation. Neither script launches PMKS, accesses a browser or reads external PMKS CSV files. Keep both scripts in the same directory for the default workflow.

## Main calculation

The main script starts with `clc; clear;`, uses row vectors and symbolic equations for the first position, and leaves its variables and tables in the workspace. Local numeric helpers use column vectors internally; histories have one row per input position.

Its sections are:

1. Input speed, geometry, CAD mass properties and loading.
2. Relative-position vectors and static equilibrium.
3. Angular velocity and acceleration loop equations.
4. Joint and mass-center velocities and accelerations.
5. Newton's second law.
6. Circle-intersection position loop and complete-cycle calculations.
7. MATLAB-only first-position tables, history CSV files and five figures.

First-position equations use `syms` and `solve`, following the lecture format. Equivalent numeric matrices handle the complete cycle. Moments are taken about each link's mass center; the independent in-plane force components and out-of-plane moment component are solved. Shared-joint compatibility, force balance, energy rate and finite differences provide numerical checks.

First-position tables contain static joint forces and input torque, dynamic joint forces and input torque, angular velocities, angular accelerations, joint velocities, joint accelerations and mass-center accelerations. There are no PMKS columns or embedded PMKS arrays in the main script.

With `show_plots = true`, five figures appear and PNG copies are saved:

1. Initial linkage and point paths.
2. Link angular velocities and angular accelerations.
3. Joint x/y velocities and accelerations.
4. Mass-center x/y accelerations.
5. Static/dynamic input torque and dynamic joint-force magnitudes.

Plots include units, axis labels and legends. The linkage outline is not a free-body diagram.

## Comparison data and limits

The comparison script contains fixed numeric arrays copied from 12 actual PMKS CSV exports captured on September 8, 2026. Its local function `embedded_pmks_data` records the column mapping and original filenames. The raw CSV files are provenance records only, not runtime dependencies.

The available references contain 361 samples over a 2.592-second cycle and support:

- 24 first-position scalar comparisons.
- 18 full-cycle component comparisons.
- AB/BC/DCE angular velocities and angular accelerations.
- B/C/E linear velocities and linear accelerations, including the original first-position magnitudes.

The comparison script writes side-by-side tables and full-cycle error metrics, saves the reference data for inspection, and produces one four-panel MATLAB/PMKS overlay figure. No phase fitting or amplitude adjustment is applied.

Unavailable references remain `NaN`, never values copied from the MATLAB solver. The EF/GF and F exports were blank, and H was absent from the reference model. Matching static-force and CAD mass-center acceleration exports are also unavailable. Newton's second-law forces and torque are not compared with PMKS.

These references require the original A-G geometry, the 2.592-second input period, and steady counterclockwise input rotation. The comparison guard disables mismatched references instead of rescaling them. A changed model requires new matching PMKS data. The newer PMKS model containing CAD properties and a 200 N load does not update these fixed arrays automatically. Differences, including larger differences in some E components, are reported rather than treated as exact agreement.

## Model assumptions

- Input: 12,500 parts in nine hours, one round trip per input revolution, constant 23.148148148 rpm and zero input angular acceleration. Startup transients and extra dwell are excluded.
- Material: 6061-T6 at 2700 kg/m^3, using the supplied CAD masses and centroidal `Lzz` values.
- Static loading: `Q = 200 N` vertically downward at H, plus link self-weight.
- Dynamic loading: a moving point-mass artifact with `m_payload = 200/g`; its force on the linkage is `Q_dynamic = m_payload*(g_vector-a_H)`, generally not purely downward.
- H is 1.843 m beyond F along G-F. The bare GFH mass includes the structural extension but excludes the artifact.

CAD mass-center coordinates were transcribed at two-decimal precision. The given D-C-E noncollinearity is retained although the pictured CAD link appears straight. Full-cycle loaded results are a load envelope, not a verified pickup/release sequence. Motor sizing, fatigue and conveyor synchronization are not certified by these calculations.

## Output files

The main script writes tables, five PNG figures, and `classroom_results.mat` to `results_matlab`.

The comparison script writes side-by-side tables, error metrics, one PNG figure, and `comparison_results.mat` to `results_pmks_comparison`.

Both output folders are created automatically beside the scripts and excluded from Git. Rerunning a script overwrites its own generated outputs. The two scripts in this folder are sufficient for the workflow; the older code versions and raw PMKS CSV files are not required.
