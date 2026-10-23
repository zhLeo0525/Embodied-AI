# Chapter 5 assignment code

## Notebooks (`notebooks/`)

| Notebook | Contents | Autograder |
|---|---|---|
| `Exp5'1-zmp-planner.ipynb` | Example 5.1: LIPM + `ZmpPlanner`, timing sweep | — |
| `Exp5'2-littledog.ipynb` | Example 5.2: LittleDog gait optimization (SNOPT) | — |
| `Exp5'8-footstep-planning.ipynb` | Section 5.8(a): MIQP footstep planning | 16/16 |
| `Exp5'8-footstep-planning-gcs.ipynb` | Section 5.8(c): GCS footstep planning | 16/16 |

The notebooks are stored with their execution outputs.

## Assignment analysis scripts (`analysis/`)

| Script | Produces |
|---|---|
| `zmp_analysis.py` | `results_zmp.json`, `fig5_1..fig5_3` |
| `littledog_analysis.py` | `results_littledog.json`, `fig5_8..fig5_10` |
| `footstep_analysis.py` | `results_footstep.json`, `fig5_4..fig5_7` |
| `zmp_exercise_analysis.py` | `results_zmp_exercise.json`, `fig5_11`, `fig5_12` |

`littledog_analysis.py` reads `littledog_walking_trot.npz`, which is written
by the LittleDog notebook.  `results_mip.json` and `results_gcs.json` are the
autograder outputs.

The companion Chinese derivation notes use their own scripts in
`Ch5/exp5_analysis/`; those files are not part of this assignment code.

## Notes

* `COLAB_TESTING=1` makes `running_as_notebook` false and prevents interactive
  visualizers from blocking headless execution.  Do **not** use it for the
  LittleDog notebook: the SNOPT iteration limits are set from
  `running_as_notebook`, and with it false the optimizer stops after one
  iteration.
* The two footstep-planning notebooks each end with a `Grader.grade_output`
  cell that writes `results.json` and prints the score.
