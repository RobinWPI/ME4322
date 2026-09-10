# ME4322 - Homework 1

Six-bar linkage analysis for Fall 2026.

## Contents

| Folder | Contents |
| --- | --- |
| [PMKS](PMKS/) | Browser link, PMKS+ import model, and model parameters |
| [SOLIDWORKS](SOLIDWORKS/) | Native SOLIDWORKS part and STEP export |
| [MATLAB](MATLAB/) | Main calculation and separate data comparison scripts |
| [Report](Report/) | Reserved for the final report |
| [Checklist](Checklist/) | Reserved for the homework checklist |

## Open the mechanism

Use the **Open the saved PMKS+ model** link in [PMKS/README.md](PMKS/README.md), preferably in Chrome. The same folder contains a Windows browser shortcut and the importable model.

After opening the saved model, restore the input speed to **23.1481481481481 RPM**, counterclockwise. The saved PMKS format rounds the speed to 23 RPM; see the PMKS instructions for precision and loading notes.

## Run the MATLAB analysis

Set MATLAB's Current Folder to `HW1/MATLAB`, then run:

```matlab
main
data_comparison_analysis
```

- `main.m` performs the MATLAB calculations, exports MATLAB-only result tables, and produces five figures. Symbolic Math Toolbox is required.
- `data_comparison_analysis.m` reads the saved MATLAB results and compares them with PMKS values hard-coded in that script. It produces comparison tables and a comparison figure; it does not rerun the mechanism calculations or call PMKS.

Running only `main` is sufficient for the MATLAB analysis. No external PMKS CSV files are needed. See [MATLAB/README.md](MATLAB/README.md) for model assumptions and reference-data limits.

## Future documents

The final report and checklist have not been uploaded yet. Their folders contain placeholders only.

The PMKS bundle previously stored at `homework1/pmks_web` has moved to `HW1/PMKS`. Existing course files outside HW1 are unchanged.
