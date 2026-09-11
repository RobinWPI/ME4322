# Homework 1 PMKS+ Model

Updated September 11, 2026. Use this model instead of earlier imports.

## Open and run

1. [Open the current PMKS+ model](https://app.pmksplus.com/?3u.BG.N,0.2112.6A,A,Lu,7b,0.0B,B,Q6,FU,0.0C,C,3-,GB,0.4D,D,4T,t,0.0E,E,33,di,0.0F,F,0FK,eA,0.4G,G,o,38,0.0H,H,0Qp,12a,0..YRAB,AB,1zU,44,O1,Bb,B3E2CD,A,B,,.YRBC,BC,4lI,q_,F5,Fq,FDCDAC,B,C,,.YREF,EF,3yW,Uo,066,dx,F4CAE4,E,F,,.YRDCE,DCE,8B8,4MI,3m,KO,CBD5E8,C,E,D,,.YRGFH,GFH,EaW,ODY,0C-,Yp,E6F5C9,G,F,H,,..2F1,GFH,Q,0Qp,12a,0Qp,wm,mr0..N_Q) in Chrome, or use **Open** in PMKS+ to import `model.pmks`.
2. Set **Settings > Input Speed = 23.1481481481481 RPM**, counterclockwise. The save format reopens at **23 RPM**; restore the exact speed after every import.
3. Select a joint or link, open **Analyze**, and use **Download Data (CSV)**. Initial-position results are the row at **0 s**. Use **Static (Equilibrium)** for the report's force comparison.

## Model settings

- Five moving links: AB, BC, DCE, EF, GFH. Ground joints: A, D, G; input: AB at A.
- SI units: m, kg, kg m², N, N m. Cycle: 2.592 s; steady input with zero angular acceleration.
- Supplied SOLIDWORKS masses, centroidal inertias, and centers of mass; 6061-T6 model.
- Q = **(0, -200) N** at H, fixed in the global frame. Self-weight is included. No extra payload mass or payload inertia is added.
- Saved H and COM coordinates have 0.001 m precision. PMKS+ uses g = 9.80665 m/s²; MATLAB uses 9.81 m/s².

Keep the supplied link-record order **AB, BC, EF, DCE, GFH**. The old order caused missing second-loop kinematics and incorrect dynamic forces in PMKS+ 2.0.3; the physical geometry is unchanged. This file works around that issue without modifying PMKS+.

At 0 s, the verified native input torques are **-504.264 N m static** and **-47.039 N m dynamic**. The latter is a diagnostic check; the report retains MATLAB-only Newton's second-law results, as required by the checklist.

`reference_csv/` contains 40 native exports. `reference_manifest.json` records their hashes, units, first rows, and exact model URL. MATLAB embeds its reference values and does not read these files at runtime.
