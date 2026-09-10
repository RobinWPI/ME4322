# Homework 1 — PMKS+ Model

## Open in a browser

**[Open the saved PMKS+ model](https://app.pmksplus.com/?3u.BG.N,0.2112.6A,A,Lu,7b,0.0B,B,Q6,FU,0.0C,C,3-,GB,0.4D,D,4T,t,0.0E,E,33,di,0.0F,F,0FK,eA,0.4G,G,o,38,0.0H,H,0Qp,12a,0..YRAB,AB,1zU,44,O1,Bb,B3E2CD,A,B,,.YRBC,BC,4lI,q_,F5,Fq,FDCDAC,B,C,,.YRDCE,DCE,8B8,4MI,3m,KO,CBD5E8,C,E,D,,.YREF,EF,3yW,Uo,066,dx,F4CAE4,E,F,,.YRGFH,GFH,EaW,ODY,0C-,Yp,E6F5C9,G,F,H,,..2F1,GFH,Q,0Qp,12a,0Qp,wm,mr0..N_Q)**

No separate website deployment is required: the URL contains the model and opens it in the PMKS+ web app. An Internet connection and the PMKS+ service are required; this is not an offline copy of PMKS+.

- On Windows, download and double-click `Open_PMKS_Model.url`, or open the link above in Chrome.
- Alternatively, open [PMKS+](https://app.pmksplus.com/), select **Open**, and choose `model.pmks`.

## Restore the exact input speed

After reopening, set **Settings → Input Speed = 23.1481481481481 RPM**, counterclockwise. The PMKS+ save format stores integer RPM, so the saved model initially opens at 23 RPM. This is a format limitation, not a change to the intended simulation speed.

Target: 12,500 parts in 9 hours, with one input revolution completing one out-and-back cycle and transferring one part. The cycle period is 2.592 s; the input angular velocity is constant at 2.42406840554768 rad/s, with zero input angular acceleration.

## Model contents

- Links: AB, BC, DCE, EF, GFH; ground joints: A, D, G; input: AB at A.
- SI metres and radians; H lies beyond F on the GF extension.
- Q = 200 N downward at H on GFH: global components (0, -200) N. The force anchor moves with GFH, but its direction stays globally vertical.
- SolidWorks masses and centroidal inertias are included, with transformed global centers of mass.

| Link | Mass (kg) | Centroidal inertia (kg·m²) | Saved CoM (m) |
|---|---:|---:|---|
| AB | 8.03 | 0.26 | (1.537, 0.741) |
| BC | 19.41 | 3.39 | (0.965, 1.012) |
| DCE | 33.48 | 17.81 | (0.240, 1.304) |
| EF | 16.16 | 1.97 | (-0.390, 2.555) |
| GFH | 59.68 | 99.17 | (-0.831, 2.227) |

## Precision and assumptions

The PMKS+ codec saves coordinates to 0.001 m. H and the Q anchor coincide exactly at saved H = (-1.715, 4.260) m. `model.json` records both saved values and higher-precision calculation targets. To restore more precise CoMs, enter their target coordinates after all joint geometry edits; PMKS+ can recalculate CoMs when joints are edited.

SolidWorks screenshot values have only two displayed decimals. Use centroidal Lzz, not inertia about the CAD origin. The current material assumption is 6061-T6 (density 2700 kg/m³); density alone does not verify alloy grade or temper. The straight CAD DCE model differs slightly from the specified joint geometry: C is about 5.49 mm off line DE.

Q is a prescribed constant external force. No payload mass has been added to GFH, and this model does not implement an artifact's acceleration-dependent inertial reaction. Link self-weight/gravity is a separate simulation setting; a downward Q alone does not enable gravity.

## Files

- `model.pmks`: prepared PMKS+ import model.
- `Open_PMKS_Model.url`: browser shortcut.
- `model.json`: portable, readable parameters and precision notes.
- `README.md`: opening instructions.

This is a prepared model import bundle, not a native browser export or a hosted copy of the PMKS+ application.
