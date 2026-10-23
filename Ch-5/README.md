# Chapter 5 -- Highly-articulated Legged Robots

This folder contains the derivation notes, the solved examples and the
Section 5.8 exercises for Chapter 5.

## Main deliverables

| File | Description |
|---|---|
| `Ch5_Highly_Articulated_Legged_Robots_深度推导笔记.md` | Full Chinese derivation notes |
| `Ch5_Highly_Articulated_Legged_Robots_深度推导笔记.pdf` | PDF (40 pages, generated from the Markdown) |
| `Ch5_Highly_Articulated_Legged_Robots_深度推导笔记_preview.html` | Self-contained HTML preview (images embedded, MathJax CDN) |
| `examples/Exp5.1-zmp-planner.ipynb` | Example 5.1, executed |
| `examples/Exp5.2-littledog.ipynb` | Example 5.2, executed (SNOPT, gait optimization) |
| `exercises/Exp5.8.1-footstep-mip.ipynb` | Exercise 5.8.1, MIQP footstep planning, **16/16** |
| `exercises/Exp5.8.3-footstep-gcs.ipynb` | Exercise 5.8.3, GCS footstep planning, **16/16** |
| `exercises/results_mip.json`, `exercises/results_gcs.json` | Autograder results |

## Assignment report (作业)

The assignment deliverable is the English LaTeX report in
`Ch5_Assignment_Report/`, in the same format as the Chapter 3 and Chapter 4
reports:

| File | Description |
|---|---|
| `Ch5_Assignment_Report/Ch5_Report.pdf` | Final report, 24 pages (Examples 5.1, 5.2 and all of Section 5.8) |
| `Ch5_Assignment_Report/Ch5_Report.tex` | LaTeX source |
| `Ch5_Assignment_Report/figures/` | Report figures |
| `Ch5_Assignment_Report/code/notebooks/` | Executed assignment notebooks (all four) |
| `Ch5_Assignment_Report/code/analysis/` | Assignment analysis scripts and result files |
| `Ch5_Assignment_Report/Ch5_Assignment_Submission.zip` | Report PDF, LaTeX, figures and code |
| `Ch5_Assignment_Report/Ch5_Assignment_Code.zip` | Code only |

The assignment notebook copies live in `Ch5_Assignment_Report/code/notebooks/`
(`Exp5'1-*`, `Exp5'2-*`, `Exp5'8-*`).  The notes notebook copies live in
`examples/` and `exercises/`.  Both sets were executed independently and
report the same autograder scores (MIQP 16/16, GCS 16/16).

## Notes structure

* 5.1 The spacecraft thought experiment
* 5.2 CoM dynamics, CoP and ZMP (full derivation)
* 5.3 ZMP-based planning (LIPM, DCM, Example 5.1)
* 5.4 Centroidal dynamics and whole-body QP (Example 5.2 LittleDog)
* 5.8 Section 5.8 exercises
  * 5.8.1 MIQP footstep planning
  * 5.8.2 Understanding the zero-moment point (analytic solution, parts a--h)
  * 5.8.3 GCS footstep planning
* Appendices: numerical checks, formula sheet, common mistakes, links to Ch. 3/4

## Figures and numerical scripts

`exp5_analysis/` contains:

| File | Purpose |
|---|---|
| `example51_zmp.py` | Reproduces Example 5.1, checks the CoP identity, timing/R sensitivity and feedback |
| `exercise52_zmp.py` | Exercise 5.2: three ZMP computations, free-leg simulation, foot-design study |
| `make_ch5_figures.py` | Generates all figures in `exp5_analysis/figures/` |
| `make_ch5_pdf.py` | Renders the Markdown to PDF with headless Chrome |
| `results_ex51_zmp.json`, `results_ex52_zmp.json` | Numerical results quoted in the notes |

## Reproduce

Use the course environment (Drake + underactuated):

```bash
PY=/Users/leo/venvs/drake/bin/python

# Analysis and figures
$PY exp5_analysis/example51_zmp.py
$PY exp5_analysis/exercise52_zmp.py
$PY exp5_analysis/make_ch5_figures.py

# Execute the notebooks headlessly
COLAB_TESTING=1 /Users/leo/venvs/drake/bin/jupyter nbconvert \
  --to notebook --execute --inplace --ExecutePreprocessor.timeout=3600 \
  examples/Exp5.1-zmp-planner.ipynb examples/Exp5.2-littledog.ipynb \
  exercises/Exp5.8.1-footstep-mip.ipynb exercises/Exp5.8.3-footstep-gcs.ipynb

# Rebuild the PDF (and HTML)
$PY exp5_analysis/make_ch5_pdf.py
```

The LittleDog notebook uses SNOPT. In `COLAB_TESTING=1` mode the original
notebook limits SNOPT to one iteration; the copy in this folder sets
`Iterations Limits = 20000` and `Major Iterations Limit = 100` so that the
optimization actually converges, and saves the result to
`exp5_analysis/ex52_littledog_solution.npz`.

## Results at a glance

| Item | Result |
|---|---|
| Example 5.1 CoP identity error | 0 (machine precision) |
| Example 5.1 ZMP tracking error (default weights) | 0.153 m |
| Example 5.1 final error after a velocity push | \(1.5\times10^{-5}\) m |
| Exercise 5.1 MIQP | cost A 4.1593, B 5.6721; 16/16 |
| Exercise 5.2 | \(\theta_H=45^\circ\), \(\theta_T=146.3^\circ\); actuated range \(76.1^\circ\)--\(99.2^\circ\) |
| Exercise 5.3 GCS | 10-edge paths; 16/16 |
| Example 5.2 LittleDog | SNOPT success, stride 0.6875 s, CoM height 0.131--0.148 m |
