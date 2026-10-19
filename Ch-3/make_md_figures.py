"""Render the TikZ diagrams of Ch3_Report.tex as standalone PNG images for
the Markdown notes.

Run with: /Users/leo/venvs/drake/bin/python make_md_figures.py
"""

import os
import re
import shutil
import subprocess


TEX = "Ch3_Assignment_Report/Ch3_Report.tex"
OUT = "md_figures"
os.makedirs(OUT, exist_ok=True)

src = open(TEX, encoding="utf-8").read()

# Each entry: (name, unique substring at the start of the TikZ picture)
markers = {
    "acrobot_schematic": r"\begin{tikzpicture}[>=Stealth,scale=1.35]",
    "cartpole_schematic": r"\begin{tikzpicture}[>=Stealth,scale=1.0]" + "\n  \\draw[thick] (-2.6,0)--(2.6,0);",
    "lqr_block": r"\begin{tikzpicture}[>=Stealth,node distance=1.5cm,",
    "planar_quadrotor": r"\begin{tikzpicture}[>=Stealth,scale=1.1]" + "\n  \\draw[very thick,black!70]",
    "linearization_local": r"\begin{tikzpicture}[>=Stealth,scale=1.05]",
    "two_step_path": r"\begin{tikzpicture}[>=Stealth,scale=1.0," + "\n  node/.style={circle",
    "controllability_geometry": r"\begin{tikzpicture}[>=Stealth,scale=1.1]" + "\n  \\draw[->] (-2.6,0)--(2.6,0)",
    "stability_flow": r"\begin{tikzpicture}[>=Stealth,node distance=0.9cm,",
}

PREAMBLE = r"""\documentclass[border=6pt]{standalone}
\usepackage[T1]{fontenc}
\usepackage{lmodern}
\usepackage{amsmath,amssymb}
\usepackage{tikz}
\usetikzlibrary{arrows.meta,positioning,calc,decorations.pathreplacing,angles,quotes}
\newcommand{\x}{\mathbf{x}}
\newcommand{\A}{\mathbf{A}}
\newcommand{\B}{\mathbf{B}}
\newcommand{\C}{\mathcal{C}}
\newcommand{\Q}{\mathbf{Q}}
\newcommand{\R}{\mathbf{R}}
\begin{document}
"""

for name, marker in markers.items():
    i = src.find(marker)
    if i < 0:
        print("MISSING", name)
        continue
    j = src.find(r"\end{tikzpicture}", i)
    tikz = src[i:j + len(r"\end{tikzpicture}")]
    tex = PREAMBLE + tikz + "\n\\end{document}\n"
    tex_path = os.path.join(OUT, name + ".tex")
    with open(tex_path, "w", encoding="utf-8") as f:
        f.write(tex)
    subprocess.run(
        ["pdflatex", "-interaction=nonstopmode", "-halt-on-error",
         os.path.basename(tex_path)],
        cwd=OUT, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    pdf_path = os.path.join(OUT, name + ".pdf")
    subprocess.run(
        ["pdftocairo", "-png", "-singlefile", "-r", "300",
         pdf_path, os.path.join(OUT, name)],
        check=True,
    )
    print("rendered", name)

# The double-pendulum geometry is a raster figure from the textbook.
shutil.copy(
    "Ch3_Assignment_Report/figures/double_pendulum_geometry.png",
    os.path.join(OUT, "double_pendulum_geometry.png"),
)
print("copied double_pendulum_geometry.png")
