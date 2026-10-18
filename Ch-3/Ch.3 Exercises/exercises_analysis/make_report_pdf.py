"""Render ../Ch3_Section3.8_作业解答.md to HTML with MathJax, then print
it to PDF with headless Google Chrome.

The HTML is written next to the markdown file so that the relative image
paths keep working.
"""

import html as html_lib
import os
import pathlib
import re
import subprocess

import mistune

REPORT_DIR = pathlib.Path("..").resolve()
MD = REPORT_DIR / "Ch3_Section3.8_作业解答.md"
HTML = REPORT_DIR / "Ch3_Section3.8_作业解答.html"
PDF = REPORT_DIR / "Ch3_Section3.8_作业解答.pdf"

CSS = """
@page { size: A4; margin: 17mm 15mm 17mm 15mm; }
body {
  font-family: "PingFang SC", "Heiti SC", "Songti SC", "Helvetica Neue", sans-serif;
  font-size: 10.4pt;
  line-height: 1.62;
  color: #1a1a1a;
  max-width: 100%;
  margin: 0;
}
h1 {
  font-size: 17pt;
  border-bottom: 2px solid #2b5c8a;
  padding-bottom: 4px;
  margin-top: 22px;
  page-break-before: always;
}
h1:first-of-type { page-break-before: avoid; }
h2 { font-size: 13.5pt; margin-top: 18px; color: #1f4e79; }
h3 { font-size: 11.8pt; margin-top: 14px; color: #1f4e79; }
p, li { orphans: 3; widows: 3; }
code {
  font-family: "SF Mono", Menlo, Consolas, monospace;
  font-size: 0.88em;
  background: #f3f4f6;
  padding: 1px 4px;
  border-radius: 3px;
}
pre {
  background: #f6f8fa;
  border: 1px solid #e1e4e8;
  border-radius: 5px;
  padding: 9px 11px;
  line-height: 1.42;
  white-space: pre-wrap;
  word-break: break-word;
  page-break-inside: avoid;
}
pre code { background: none; padding: 0; }
table {
  border-collapse: collapse;
  width: 100%;
  margin: 10px 0 14px 0;
  font-size: 9.3pt;
  page-break-inside: avoid;
}
th, td {
  border: 1px solid #b9c6d3;
  padding: 4px 6px;
  vertical-align: top;
}
th { background: #e8eef5; text-align: left; }
blockquote {
  border-left: 4px solid #9db8d2;
  background: #f5f8fb;
  margin: 10px 0;
  padding: 6px 12px;
  color: #33475b;
}
img {
  max-width: 100%;
  display: block;
  margin: 8px auto;
  page-break-inside: avoid;
}
hr { border: none; border-top: 1px solid #d0d7de; margin: 16px 0; }
.meta { color: #555; font-size: 9.3pt; }
"""

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>Ch3 Section 3.8 作业解答</title>
<script>
window.MathJax = {{
  tex: {{
    inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
    displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
    processEscapes: true
  }},
  svg: {{ fontCache: 'global' }},
  options: {{ enableMenu: false }}
}};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>
<style>{css}</style>
</head>
<body>
{body}
</body>
</html>
"""


def protect_math(text):
    """Hide LaTeX from the Markdown parser.

    Markdown treats ``\\\\`` as an escaped backslash, which destroys the row
    separators of LaTeX matrices (and also eats escapes such as ``\\{``).
    Each math segment is replaced by a plain token before the Markdown
    renderer runs, then restored afterwards.
    """
    tokens = {}

    def stash(value):
        key = f"MATHTOKEN{len(tokens)}END"
        tokens[key] = value
        return key

    text = re.sub(
        r"\$\$.*?\$\$", lambda m: "\n\n" + stash(m.group(0)) + "\n\n",
        text, flags=re.S,
    )
    text = re.sub(
        r"(?<!\$)\$(?!\$).+?(?<!\$)\$(?!\$)",
        lambda m: stash(m.group(0)),
        text,
        flags=re.S,
    )
    return text, tokens


def main():
    text = MD.read_text(encoding="utf-8")
    text, math_tokens = protect_math(text)
    renderer = mistune.create_markdown(escape=False, plugins=["table"])
    body = renderer(text)
    for key, value in math_tokens.items():
        body = body.replace(key, html_lib.escape(value, quote=False))
    html = HTML_TEMPLATE.format(css=CSS, body=body)
    HTML.write_text(html, encoding="utf-8")

    chrome = (
        "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
    )
    cmd = [
        chrome,
        "--headless=new",
        "--disable-gpu",
        "--no-sandbox",
        "--allow-file-access-from-files",
        "--virtual-time-budget=45000",
        "--no-pdf-header-footer",
        f"--print-to-pdf={PDF}",
        HTML.as_uri(),
    ]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    print("PDF written to", PDF)
    print("HTML written to", HTML)


if __name__ == "__main__":
    main()
