# Chapter 4 Assignment -- Report and Code

This folder contains the consolidated submission for Chapter 4
(Simple Models of Walking and Running):

* `Ch4_Report.pdf` -- final report (16 pages, English).
* `Ch4_Report.tex` -- LaTeX source.
* `figures/` -- figures used in the report.
* `code/` -- Python scripts, analysis code and executed notebooks
  (see `code/README.md`).
* `Ch4_Assignment_Code.zip` -- code only.
* `Ch4_Assignment_Submission.zip` -- report PDF, LaTeX, figures and code.

## Contents of the report

Part I (Examples):

* Example 4.3 -- Rimless wheel: hybrid dynamics, return map, fixed point and
  slope sensitivity.
* Example 4.4 -- Compass gait: passive limit cycle, step-to-step map and
  initial-condition/slope sensitivity.
* Example 4.5 -- SLIP simulation: flight/stance phases and energy conservation.
* Example 4.6 -- SLIP apex-to-apex map: stable/unstable fixed points,
  energy/touchdown-angle sensitivity and partial stability.

Part II (Section 4.6):

* One-dimensional hopper: energy analysis, preload controller, autograder
  result 13/13, target-height and initial-height sensitivity.

## Compile

```bash
pdflatex Ch4_Report.tex
pdflatex Ch4_Report.tex
```

## Before submitting

Fill in your name and student ID on the title page of `Ch4_Report.tex`
(currently `Leo` and a blank student-ID line) and recompile.
