# Chapter 1 -- Manipulation Assignment: Code

Environment: Drake 1.58.0, `manipulation` 2026.10.17, Python 3.12
(`/Users/leo/venvs/drake/bin/python`).

## Notebooks (`notebooks/`)

* `Exp1.1-1.2-teleoperation.ipynb` -- Examples 1.1 and 1.2
  (2D/3D teleoperation).  Example 1.3 (System Diagrams) is discussed in the
  report; it is a conceptual/interactive example without a separate notebook.
* `Exp1.7.2-drake-systems.ipynb` -- Drake Systems Fundamentals, 70/70.
* `Exp1.7.3-physics-simulation.ipynb` -- IIWA14, controller, table and
  letters; autograders 50/50.
* `Exp1.7.4-hardwarestation.ipynb` -- HardwareStation and scenarios, 50/50.

The notebooks use the pinned `drake_models` package (IIWA14, WSG and the
YCB mustard bottle).  A local fallback IIWA URDF is kept in `models/` for
offline use.  Run them from `notebooks/` with:

```bash
COLAB_TESTING=1 jupyter nbconvert --to notebook --execute --inplace \
  --ExecutePreprocessor.timeout=300 Exp1.7.2-drake-systems.ipynb
```

## Analysis (`analysis/`)

`make_ch1_figures.py` regenerates the pendulum and IIWA figures used in the
report.  Run it from `code/` (it expects `models/` next to it) or adjust the
model path.

## Report

Compile `../Ch1_Report.tex` with `pdflatex` twice.  Figures are in
`../figures/`.
