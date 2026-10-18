# Chapter 3 Assignment -- Report and Code

This folder contains the consolidated submission for Chapter 3
(Acrobots, Cart-Poles, and Quadrotors):

* `Ch3_Report.pdf` -- final report (25 pages, English).
* `Ch3_Report.tex` -- LaTeX source of the report.
* `figures/` -- all figures used in the report.
* `make_report_figures.py` -- regenerates the English-labelled figures from
  the JSON results of the analysis scripts.
* `code/` -- Python scripts and notebooks (see `code/README.md`).
* `Ch3_Assignment_Submission.zip` -- report PDF, LaTeX source, figures and
  code in one archive (created locally; not tracked in git if too large).

## Compile the report

```bash
pdflatex Ch3_Report.tex
pdflatex Ch3_Report.tex
```

## Contents of the report

Part I (Examples):

* Example 3.1 -- Dynamics of the Acrobot
* Example 3.2 -- Dynamics of the Cart-Pole System
* Example 3.5 -- LQR for the Acrobot and Cart-Pole
* Example 3.6 -- LQR for Quadrotors (planar + 3D)

Part II (Section 3.8):

* Exercise 3.1 -- Cart-Pole linearisation and balancing
* Exercise 3.2 -- Cart-pole URDFs and LQR
* Exercise 3.3 -- Controllability of discrete and continuous LTI systems
* Exercise 3.4 -- Nonlinear stability vs. linearisation

Each example section contains the model, the linearisation/LQR design, the
simulation results, a theoretical interpretation and a sensitivity study.
The Section 3.8 part contains the derivations, the code snippets and the
relevant plots.

## Before submitting

Fill in your name and student ID in the title page of `Ch3_Report.tex`
(currently Zihao Liu / 220261979) and recompile.
