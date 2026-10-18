# Chapter 4 Assignment -- Code

This folder contains the Python code used for `../Ch4_Report.pdf`.

## Environment

```
Python 3.12
drake 1.58.0
underactuated 2026.9.8
numpy 2.5.4
scipy 1.18.1
matplotlib 3.11.2
```

The results were produced with `/Users/leo/venvs/drake/bin/python`.

## Analysis scripts

Run from `analysis/`:

```bash
cd analysis
python rimless_wheel_analysis.py   # Example 4.3: return map and fixed points
python compass_gait_analysis.py    # Example 4.4: limit cycle and sensitivity
python slip_analysis.py            # Examples 4.5/4.6: apex-to-apex map
python one_d_hopper_analysis.py    # Section 4.6: hopping controller
```

The SLIP and hopper scripts read the corresponding notebooks from
`../notebooks/` and execute their plant/controller definitions.  Each script
writes `results_*.json` and a `figures/` folder in the current directory.

## Notebooks

`notebooks/` contains the supplied notebooks, executed and saved with outputs:

* `Exp4'3-rimless-wheel.ipynb`
* `Exp4'4-compass-gait.ipynb`
* `Exp4'5-6-slip.ipynb`
* `Exp4'6-one_d_hopper.ipynb` -- Section 4.6, autograder **13/13**.

Headless execution:

```bash
COLAB_TESTING=1 jupyter nbconvert --to notebook --execute --inplace \
  --ExecutePreprocessor.timeout=300 "Exp4'6-one_d_hopper.ipynb"
```

## Report

Compile `../Ch4_Report.tex` with `pdflatex` twice.  The figures used by the
report are in `../figures/`.
