# Chapter 3 Assignment -- Code

This folder contains the Python code used for the report
`../Ch3_Report.pdf`.  The authoritative analysis scripts are also kept in the
original repository folders; the copies here are organized for submission.

## Environment

```
Python 3.12
drake 1.58.0
underactuated 2026.9.8
numpy 2.5.4
scipy 1.18.1
matplotlib 3.11.2
```

The environment used for the results is `/Users/leo/venvs/drake/bin/python`.

## Examples (Part I)

Run from `examples/`:

```bash
cd examples
python exp1_analysis.py          # Example 3.1: acrobot dynamics, LQR, sensitivity
python cp_analysis.py            # Example 3.2: cart-pole dynamics, LQR, sensitivity
python lqr_comparison.py         # Example 3.5: LQR comparison
python quadrotor_analysis.py     # Example 3.6: planar + 3D quadrotor LQR
```

Each script writes `results.json` and `figures/` in the current directory.
`lqr_comparison.py` imports the helpers from `exp1_analysis.py` and
`cp_analysis.py`, which are in the same folder.

## Section 3.8 (Part II)

Run from `section3_8/`:

```bash
cd section3_8
python ex3_1_balancing_analysis.py       # Exercise 3.1
python ex3_2_urdf_lqr_analysis.py        # Exercise 3.2 (reads ../notebooks/cartpoles_urdf.ipynb)
python ex3_3_controllability.py          # Exercise 3.3
python ex3_4_stability_linearization.py  # Exercise 3.4
```

The two notebooks with autograders are in `notebooks/`:

* `cartpole_balancing.ipynb` -- Exercise 3.1, autograder 13/13.
* `cartpoles_urdf.ipynb` -- Exercise 3.2, autograder 12/12.

They can be executed headlessly with:

```bash
COLAB_TESTING=1 jupyter nbconvert --to notebook --execute --inplace \
  --ExecutePreprocessor.timeout=180 cartpoles_urdf.ipynb
```

`COLAB_TESTING=1` makes `running_as_notebook` false so that the MeshCat
visualizer does not block.

## Example notebooks

`notebooks/` also contains the textbook example notebooks that were run:

* `Exp3'1-acrobot.ipynb`
* `Exp3'2-cartpole.ipynb`
* `Exp3'5-acrobot.ipynb`
* `Exp3'5-cartpole.ipynb`
* `Exp3'6-planar-quadrotor.ipynb`
* `Exp3'6-quadrotor.ipynb`

The planar quadrotor notebook `Exp3'6-planar-quadrotor.ipynb` was executed and
its animation is stored in the notebook.  The quantitative sensitivity study
for Example 3.6 is in `examples/quadrotor_analysis.py`.

## Report

The LaTeX source is `../Ch3_Report.tex`; compile with:

```bash
cd ..
pdflatex Ch3_Report.tex
pdflatex Ch3_Report.tex
```

`make_report_figures.py` regenerates the English-labelled figures used by the
report from the JSON results.
