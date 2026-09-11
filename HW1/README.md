# ME4322 Homework 1

Final model, MATLAB code, report links and supporting sections, updated September 11, 2026.

- [Essay (Google Docs)](https://docs.google.com/document/d/1jJHeWnsE9gbaxTjAvMku-VIbA_qBt6lieXLE9j1jCU0/edit?tab=t.0)
- [Checklist (Google Docs)](https://docs.google.com/document/d/1Ascv-7TABrtnUP82rIQxwvR_fvaNIsdAwOIh9Mvc8PA/edit?tab=t.0)

These Google Docs are linked only; no Word or PDF exports are included.

| Folder | Contents |
| --- | --- |
| [PMKS](PMKS/) | Corrected browser model, opening instructions and native reference exports |
| [MATLAB](MATLAB/) | Main calculation, separate comparison script and validation tests |
| [SOLIDWORKS](SOLIDWORKS/) | Original native part and STEP model |
| [Report](Report/) | Essay link, first-position tables, seven explained plots, figure images and result data |
| [Checklist](Checklist/) | Google Docs checklist link |

## Start

1. Open the model using [PMKS/README.md](PMKS/README.md). Restore **23.1481481481481 RPM counterclockwise** after importing; the save format reopens at 23 RPM.
2. In MATLAB, set Current Folder to `HW1/MATLAB`, then run [main.m](MATLAB/main.m) followed by [data_comparison_analysis.m](MATLAB/data_comparison_analysis.m). Figures appear automatically.
3. Open the essay and checklist using the Google Docs links above. Supporting tables and figure explanations remain available in [Report](Report/).

The model uses the supplied 6061-T6 CAD mass properties and a constant global **200 N downward force at H**, with link self-weight and no added payload mass. The current comparison contains 95 initial-position PMKS values and 67 complete-cycle component comparisons. Newton's second-law report results remain MATLAB-only.

The supplied PMKS link-record order fixes the old second-loop import issue. Full details and known precision differences are in the PMKS and MATLAB READMEs. The Word files are report sections, not a complete submitted paper.
