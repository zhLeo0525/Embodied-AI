# Chapter 1 -- Manipulation Assignment: Code

Environment: Drake 1.58.0, `manipulation` 2026.10.17, Python 3.12
(`/Users/leo/venvs/drake/bin/python`).

## Notebooks (`notebooks/`)

* `Exp1.2-1.3-teleoperation.ipynb` -- Examples 1.2 and 1.3
  (2D/3D teleoperation).
* `Exp1.7.2-drake-systems.ipynb` -- Drake Systems Fundamentals, 70/70.
* `Exp1.7.3-physics-simulation.ipynb` -- IIWA14, controller, table and
  letters; visible autograders 30/30.
* `Exp1.7.4-hardwarestation.ipynb` -- HardwareStation and scenarios, 50/50.

The notebooks use the local IIWA14 URDF in `models/` (same inertial
parameters as the pinned `drake_models` model, visual meshes removed so that
the exercises can run offline).  Run them from `notebooks/` with:

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
