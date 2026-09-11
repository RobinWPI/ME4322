# ME4322 Homework 1

Six-bar linkage models, MATLAB calculations, comparison data and report materials. Updated September 11, 2026.

## Essay and checklist

- [Essay (Google Docs)](https://docs.google.com/document/d/1jJHeWnsE9gbaxTjAvMku-VIbA_qBt6lieXLE9j1jCU0/edit?tab=t.0)
- [Checklist (Google Docs)](https://docs.google.com/document/d/1Ascv-7TABrtnUP82rIQxwvR_fvaNIsdAwOIh9Mvc8PA/edit?tab=t.0)

Open these links in a browser. The essay and checklist are maintained in Google Docs; this repository stores their links only, not Word or PDF exports. The Word files in `Report` are separate supporting sections.

## Download and run

Download the repository using **Code > Download ZIP** on GitHub and extract it, or clone the repository. Keep the `HW1` folder structure intact. Use Chrome for PMKS+, MATLAB with Symbolic Math Toolbox for the calculations, and SOLIDWORKS for the native CAD part.

1. Set MATLAB's Current Folder to the downloaded `HW1/MATLAB` folder.
2. Run `main` for the MATLAB calculations, result tables and five figures.
3. Optionally run `data_comparison_analysis` afterward for the PMKS+ comparisons and seven report figures.
4. Open PMKS+ separately using the model instructions below if you want to inspect the mechanism or obtain new reference exports. Neither MATLAB script requires PMKS+ to be running.

## MATLAB files

| File | How to use it |
| --- | --- |
| [main.m](MATLAB/main.m) | Run independently in MATLAB. It solves the linkage kinematics and static and dynamic loads using MATLAB only, without PMKS+ reference data. |
| [data_comparison_analysis.m](MATLAB/data_comparison_analysis.m) | Run after `main`. It reads the saved MATLAB results, compares them with hard-coded PMKS+ exports, and produces comparison tables and plots. |
| [MATLAB README](MATLAB/README.md) | Read for the distinction between the calculation script and the AI-written analysis script. |

`main` creates `MATLAB/results_matlab/`, containing CSV tables, PNG plots and `classroom_results.mat`. The comparison script reads that MAT file and creates `MATLAB/results_pmks_comparison/`, containing comparison CSV files, PNG plots and `comparison_results.mat`. Figures also appear in MATLAB. These folders are generated locally and excluded from Git; rerunning the scripts overwrites their corresponding output files. Rerun `main` after changing its inputs before running the comparison.

## PMKS+ files

1. Open the model link in the [PMKS README](PMKS/README.md) in Chrome. Alternatively, open PMKS+ and use **Open** to import [model.pmks](PMKS/model.pmks).
2. After every import, set **Settings > Input Speed** to **23.1481481481481 RPM, counterclockwise**. The supplied save format reopens at 23 RPM, so restore the exact value before comparing results.
3. Select a joint or link and open **Analyze**. Use **Download Data (CSV)** to export a quantity. The row at **0 s** is the initial position; use **Static (Equilibrium)** for the report's force comparison.

| File or folder | How to use it |
| --- | --- |
| [Open_PMKS_Model.url](PMKS/Open_PMKS_Model.url) | Download and double-click this Windows shortcut to open the saved model in the default browser. Use the link in the PMKS README if Chrome is not the default. |
| [model.pmks](PMKS/model.pmks) | Import into PMKS+ using **Open**. Keep the supplied link-record order. |
| [model.json](PMKS/model.json) | Open in a text editor to inspect model settings, geometry, units and source information. It is a reference description; use `model.pmks` for import. |
| [reference_csv](PMKS/reference_csv/) | Open CSV files in a spreadsheet or text editor to inspect native PMKS+ exports. Open PNG files as images to view native PMKS+ chart captures. |
| [reference_manifest.json](PMKS/reference_manifest.json) | Open in a text editor to inspect export units, first rows, source model information and file hashes. |

In the reference filenames, `v_` and `a_` identify linear velocity and acceleration; `omega_` and `alpha_` identify angular velocity and acceleration; `static_F_` and `static_T` identify equilibrium joint forces and input torque. `a_S1` through `a_S5` are mass-center accelerations. `dynamic_T_diagnostic.csv` is retained for diagnosis only, not as a report comparison of Newton's second-law results. The MATLAB comparison script does not read these CSV files at runtime.

## SOLIDWORKS files

- [Linkage Part.SLDPRT](SOLIDWORKS/Linkage%20Part.SLDPRT): open in SOLIDWORKS, select the intended link configuration, and inspect or edit the part. Use **Mass Properties** to inspect mass, center of mass and inertia; check the configuration, material and units first. For planar dynamics, use centroidal inertia about the axis normal to the linkage plane.
- [Linkage Part.STEP](SOLIDWORKS/Linkage%20Part.STEP): import into SOLIDWORKS or another STEP-compatible CAD program to inspect the exchanged geometry. Use the native SLDPRT when you need the original configurations and feature history.
- [SOLIDWORKS README](SOLIDWORKS/README.md): read for the supplied files' provenance.

## Report and checklist files

| File or folder | How to use it |
| --- | --- |
| [Report README](Report/README.md) | Open the essay link and find the supporting report materials. |
| [HW1_First_Position_IEEE_Tables.docx](Report/HW1_First_Position_IEEE_Tables.docx) | Download and open in Word to copy or edit the six first-position tables. |
| [HW1_Plots_of_Various_Quantities_IEEE.docx](Report/HW1_Plots_of_Various_Quantities_IEEE.docx) | Download and open in Word to copy the seven report figures and their explanations. |
| [Figures](Report/Figures/) | Open the PNG images for viewing or insert them directly into the essay. Filenames identify angular motion, H-point motion, mass-center acceleration and torque plots. |
| [Data](Report/Data/) | Open `FirstPosition_*.csv` to inspect the tabulated initial results and `PMKS_FullCycle_Comparison.csv` to inspect full-cycle comparison errors. |
| [Checklist README](Checklist/README.md) | Follow the Google Docs checklist link and use it to check the report requirements. |

The files in `Report` are saved snapshots. Running MATLAB does not automatically replace them or update the Google Docs essay. After changing the model, regenerate and check the outputs before updating report tables and figures.

## AI contribution disclosure

The data analysis and comparison script, `data_comparison_analysis.m`, was written by AI (OpenAI Codex). PMKS+ results were exported beforehand and embedded as hard-coded numerical arrays. The AI-written script compares those arrays with the results saved by `main.m`, calculates differences, and generates the comparison tables and report plot images. Its role is post-processing, data comparison and figure generation; it does not run PMKS+ or independently solve the linkage equations. The plots are rendered by MATLAB from numerical results, not generated as illustrative images by an AI image model. Native PMKS+ chart captures are retained separately in `PMKS/reference_csv`.

## Model and comparison notes

The model uses the supplied 6061-T6 CAD mass properties, link self-weight and a constant global **200 N downward force at H**, with no added payload mass. Input rotation is steady, with zero input angular acceleration. The comparison includes 95 initial-position PMKS+ values and 67 full-cycle component comparisons. Newton's second-law report results remain MATLAB-only.

The embedded PMKS+ dataset is fixed. Changing geometry, speed, mass properties or loading requires matching new exports and an updated embedded dataset for a valid comparison. Precision differences and the required link-record order are documented in the [PMKS README](PMKS/README.md).

`HW1/.gitignore` excludes generated results and temporary files; `HW1/.gitattributes` controls Git file handling. Neither file needs to be opened or run for the analysis.
