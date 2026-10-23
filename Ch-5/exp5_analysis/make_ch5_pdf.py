"""Render the Chapter 5 notes to HTML (self-contained images) and PDF.

MathJax is loaded from the CDN; headless Chrome waits for it before printing.
"""

import base64
import html as html_lib
import os
import pathlib
import re
import shutil
import subprocess
import tempfile

import mistune

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
MD = ROOT / "Ch5_Highly_Articulated_Legged_Robots_深度推导笔记.md"
HTML = ROOT / "Ch5_Highly_Articulated_Legged_Robots_深度推导笔记.html"
PDF = ROOT / "Ch5_Highly_Articulated_Legged_Robots_深度推导笔记.pdf"

CSS = """
@page { size: A4; margin: 16mm 15mm 16mm 15mm; }
body {
  font-family: "PingFang SC", "Heiti SC", "Songti SC", "Helvetica Neue", sans-serif;
  font-size: 10.2pt; line-height: 1.62; color: #1a1a1a;
  max-width: 100%; margin: 0;
}
h1 { font-size: 18pt; border-bottom: 2px solid #2b5c8a; padding-bottom: 4px;
     margin-top: 20px; }
h2 { font-size: 13.8pt; margin-top: 18px; color: #1f4e79; }
h3 { font-size: 11.8pt; margin-top: 14px; color: #1f4e79; }
h4 { font-size: 10.6pt; margin-top: 11px; color: #33475b; }
p, li { orphans: 3; widows: 3; }
code { font-family: "SF Mono", Menlo, Consolas, monospace; font-size: 0.88em;
       background: #f3f4f6; padding: 1px 4px; border-radius: 3px; }
pre { background: #f6f8fa; border: 1px solid #e1e4e8; border-radius: 5px;
      padding: 9px 11px; line-height: 1.42; white-space: pre-wrap;
      word-break: break-word; page-break-inside: avoid; }
pre code { background: none; padding: 0; }
table { border-collapse: collapse; width: 100%; margin: 10px 0 14px 0;
        font-size: 9.2pt; page-break-inside: avoid; }
th, td { border: 1px solid #b9c6d3; padding: 4px 6px; vertical-align: top; }
th { background: #e8eef5; text-align: left; }
blockquote { border-left: 4px solid #9db8d2; background: #f5f8fb;
             margin: 10px 0; padding: 6px 12px; color: #33475b; }
img { max-width: 100%; display: block; margin: 8px auto;
      page-break-inside: avoid; }
hr { border: none; border-top: 1px solid #d0d7de; margin: 16px 0; }
"""

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>第 5 章 高自由度足式机器人 —— 深度推导笔记</title>
<script>
window.MathJax = {{
  tex: {{inlineMath: [['$','$']], displayMath: [['$$','$$']], processEscapes: true}},
  svg: {{fontCache: 'global'}},
  options: {{enableMenu: false}}
}};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>
<style>{css}</style></head><body>
{body}
</body></html>
"""


def protect_math(text):
    tokens = {}

    def stash(value):
        key = f"MATHTOKEN{len(tokens)}END"
        tokens[key] = value
        return key

    text = re.sub(r"\$\$.*?\$\$", lambda m: "\n\n" + stash(m.group(0)) + "\n\n",
                  text, flags=re.S)
    text = re.sub(r"(?<!\$)\$(?!\$).+?(?<!\$)\$(?!\$)",
                  lambda m: stash(m.group(0)), text, flags=re.S)
    return text, tokens


def embed_images(text):
    pattern = re.compile(r"!\[([^\]]*)\]\(([^)]+)\)")

    def repl(m):
        alt, rel = m.group(1), m.group(2)
        path = os.path.normpath(os.path.join(ROOT, rel))
        if not os.path.exists(path):
            return m.group(0)
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, os.path.basename(path))
            shutil.copy(path, out)
            data = base64.b64encode(open(out, "rb").read()).decode()
        return f'<img alt="{html_lib.escape(alt)}" src="data:image/png;base64,{data}" />'

    return pattern.sub(repl, text)


def main():
    text = MD.read_text(encoding="utf-8")
    text = embed_images(text)
    text, math_tokens = protect_math(text)
    renderer = mistune.create_markdown(escape=False, plugins=["table"])
    body = renderer(text)
    for key, value in math_tokens.items():
        body = body.replace(key, html_lib.escape(value, quote=False))
    HTML.write_text(HTML_TEMPLATE.format(css=CSS, body=body), encoding="utf-8")

    chrome = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
    cmd = [
        chrome,
        "--headless=new",
        "--disable-gpu",
        "--no-sandbox",
        "--allow-file-access-from-files",
        "--virtual-time-budget=60000",
        "--no-pdf-header-footer",
        f"--print-to-pdf={PDF}",
        HTML.as_uri(),
    ]
    print("Running Chrome ...")
    subprocess.run(cmd, check=True)
    print("PDF:", PDF, PDF.stat().st_size / 1e6, "MB")
    print("HTML:", HTML, HTML.stat().st_size / 1e6, "MB")


if __name__ == "__main__":
    main()
