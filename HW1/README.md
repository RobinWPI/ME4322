# ME4322 Homework 1

Final model, MATLAB code and report sections, updated September 11, 2026.

| Folder | Contents |
| --- | --- |
| [PMKS](PMKS/) | Corrected browser model, opening instructions and native reference exports |
| [MATLAB](MATLAB/) | Main calculation, separate comparison script and validation tests |
| [SOLIDWORKS](SOLIDWORKS/) | Original native part and STEP model |
| [Report](Report/) | First-position tables, seven explained plots, figure images and result data |
| [Checklist](Checklist/) | Reserved for the completed assignment checklist |

## Start

1. Open the model using [PMKS/README.md](PMKS/README.md). Restore **23.1481481481481 RPM counterclockwise** after importing; the save format reopens at 23 RPM.
2. In MATLAB, set Current Folder to `HW1/MATLAB`, then run `main` followed by `data_comparison_analysis`. Figures appear automatically.
3. Copy the editable tables and figure explanations from the two Word documents in [Report](Report/) into the final paper.

The model uses the supplied 6061-T6 CAD mass properties and a constant global **200 N downward force at H**, with link self-weight and no added payload mass. The current comparison contains 95 initial-position PMKS values and 67 complete-cycle component comparisons. Newton's second-law report results remain MATLAB-only.

The supplied PMKS link-record order fixes the old second-loop import issue. Full details and known precision differences are in the PMKS and MATLAB READMEs. The Word files are report sections, not a complete submitted paper.
