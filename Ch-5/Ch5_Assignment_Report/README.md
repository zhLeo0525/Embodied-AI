# Chapter 5 Assignment — Highly-articulated Legged Robots

This folder contains the LaTeX/PDF report for Chapter 5 of the course,
covering Example 5.1, Example 5.2 and all exercises of Section 5.8.

## Deliverables

* `Ch5_Report.pdf` — final report (24 pages, English).
* `Ch5_Report.tex` — LaTeX source.
* `figures/` — figures used in the report.
* `code/notebooks/` — the four executed notebooks:
  * `Exp5'1-zmp-planner.ipynb` — Example 5.1 (ZMP planner)
  * `Exp5'2-littledog.ipynb` — Example 5.2 (LittleDog gait optimization)
  * `Exp5'8-footstep-planning.ipynb` — Section 5.8(a) MIQP
  * `Exp5'8-footstep-planning-gcs.ipynb` — Section 5.8(c) GCS
* `code/analysis/` — standalone analysis scripts and their result files.
* `Ch5_Assignment_Code.zip` — code only.
* `Ch5_Assignment_Submission.zip` — report PDF, LaTeX, figures and code.

## Contents of the report

Part I (Examples):

* Example 5.1 — LIPM/ZMP model, Drake `ZmpPlanner`, nominal plan, footstep
  timing sensitivity, feedback-policy disturbance rejection.
* Example 5.2 — LittleDog direct transcription with centroidal dynamics,
  full kinematics and contact forces; walking-trot gait, stance schedule,
  CoM and foot trajectories, side-view snapshots.

Part II (Section 5.8):

* (a) Footstep planning via MIQP: big-M constraints, one stone per foot,
  squared step-length objective, terrains A/B, step-span sensitivity and the
  lateral-to-bridge route switch.
* (b) Understanding the ZMP: free-body analysis, stability, unactuated ZMP
  `x = -h cot(theta)`, toe/heel boundaries, actuated posture range and foot
  design.
* (c) Footstep planning via GCS: vertices as copies of stones, edge
  constraints, shortest-path convex relaxation, comparison with MIQP.

Autograder results:

```
MIQP footstep planning: 16/16
GCS  footstep planning: 16/16
```

## Environment

Drake 1.58.0 with `underactuated` 2026.9.8:

```bash
/Users/leo/venvs/drake/bin/python
```

SNOPT is available in this build and is used for the LittleDog optimization
and for the GCS convex relaxation.

## Reproduce

```bash
cd code/analysis
/Users/leo/venvs/drake/bin/python zmp_analysis.py
/Users/leo/venvs/drake/bin/python littledog_analysis.py
/Users/leo/venvs/drake/bin/python footstep_analysis.py
/Users/leo/venvs/drake/bin/python zmp_exercise_analysis.py
```

The LittleDog analysis reads `littledog_walking_trot.npz`, produced by the
executed notebook.  The notebook was run **without** `COLAB_TESTING` so that
SNOPT gets the full iteration budget:

```bash
/Users/leo/venvs/drake/bin/jupyter nbconvert --to notebook --execute \
  --inplace --ExecutePreprocessor.timeout=3000 \
  "code/notebooks/Exp5'2-littledog.ipynb"
```

The other notebooks can be executed headlessly with `COLAB_TESTING=1`.

## Compile

```bash
pdflatex Ch5_Report.tex
pdflatex Ch5_Report.tex
```

## Relation to the Chapter 5 notes

This folder is the assignment deliverable (English, same format as the
Chapter 3 and Chapter 4 reports).  The companion Chinese derivation notes are
kept at the Chapter 5 root:

* `Ch5_Highly_Articulated_Legged_Robots_深度推导笔记.md`
* `Ch5_Highly_Articulated_Legged_Robots_深度推导笔记.pdf`
* `Ch5_Highly_Articulated_Legged_Robots_深度推导笔记_preview.html`

The notes use their own scripts in `Ch5/exp5_analysis/`
(`example51_zmp.py`, `exercise52_zmp.py`, `make_ch5_figures.py`,
`make_ch5_pdf.py`), while the assignment analysis scripts are the four files
`zmp_analysis.py`, `littledog_analysis.py`, `footstep_analysis.py` and
`zmp_exercise_analysis.py` in `code/analysis/`.  Both sets of executed
notebooks are kept (the notes version in `Ch5/examples/` and
`Ch5/exercises/`, the assignment version in `code/notebooks/`).

For convenience, copies of the note-specific scripts and figures that were
originally found in this folder are collected under `notes_reference/`; the
assignment report itself uses only the 12 figures in `figures/` and the four
scripts in `code/analysis/`.
