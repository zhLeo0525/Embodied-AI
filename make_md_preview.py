"""Build self-contained HTML previews of the Markdown notes.

Images are embedded as base64 data URIs, so the preview works even if the
Markdown viewer resolves relative paths from the workspace root.  Math is
rendered with MathJax from the CDN.
"""

import base64
import html as html_lib
import os
import re
import shutil
import subprocess
import tempfile

import mistune

FILES = [
    "Ch-3/Ch3_Acrobot_CartPole_Quadrotor_深度推导笔记.md",
    "Ch-3/Ch.3 Exercises/Ch3_Section3.8_作业解答.md",
    "Ch-4/Ch4_Simple_Models_知识点.md",
]

CSS = """
@page { size: A4; margin: 16mm 15mm; }
body {
  font-family: "PingFang SC", "Heiti SC", "Songti SC", "Helvetica Neue", sans-serif;
  font-size: 11pt; line-height: 1.65; color: #1a1a1a;
  max-width: 980px; margin: 0 auto; padding: 24px;
}
h1 { font-size: 20pt; border-bottom: 2px solid #2b5c8a; padding-bottom: 4px; }
h2 { font-size: 15pt; color: #1f4e79; margin-top: 24px; }
h3 { font-size: 12.5pt; color: #1f4e79; }
code { font-family: "SF Mono", Menlo, monospace; background: #f3f4f6; padding: 1px 4px; border-radius: 3px; }
pre { background: #f6f8fa; border: 1px solid #e1e4e8; border-radius: 5px; padding: 9px 11px;
      white-space: pre-wrap; word-break: break-word; }
table { border-collapse: collapse; width: 100%; margin: 10px 0 14px 0; font-size: 9.5pt; }
th, td { border: 1px solid #b9c6d3; padding: 4px 6px; vertical-align: top; }
th { background: #e8eef5; text-align: left; }
blockquote { border-left: 4px solid #9db8d2; background: #f5f8fb; margin: 10px 0; padding: 6px 12px; color: #33475b; }
img { max-width: 100%; display: block; margin: 12px auto; }
"""

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>{title}</title>
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


def embed_images(text, md_dir, tmp_dir):
    pattern = re.compile(r"!\[([^\]]*)\]\(([^)]+)\)")

    def repl(m):
        alt, rel = m.group(1), m.group(2)
        path = os.path.normpath(os.path.join(md_dir, rel))
        if not os.path.exists(path):
            return m.group(0)
        # Resize large raster figures for the HTML preview.
        out = os.path.join(tmp_dir, os.path.basename(path))
        try:
            subprocess.run(["/usr/bin/sips", "-Z", "1100", path, "--out", out],
                           check=True, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
        except Exception:
            shutil.copy(path, out)
        data = base64.b64encode(open(out, "rb").read()).decode()
        return f'<img alt="{html_lib.escape(alt)}" src="data:image/png;base64,{data}" />'

    return pattern.sub(repl, text)


def build(md_path):
    md_dir = os.path.dirname(md_path) or "."
    text = open(md_path, encoding="utf-8").read()
    with tempfile.TemporaryDirectory() as tmp:
        text = embed_images(text, md_dir, tmp)
        text, math_tokens = protect_math(text)
        renderer = mistune.create_markdown(escape=False, plugins=["table"])
        body = renderer(text)
        for key, value in math_tokens.items():
            body = body.replace(key, html_lib.escape(value, quote=False))
    title = os.path.splitext(os.path.basename(md_path))[0]
    out = os.path.join(md_dir, title + "_preview.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(HTML_TEMPLATE.format(title=html_lib.escape(title), css=CSS, body=body))
    print("wrote", out, f"{os.path.getsize(out)/1e6:.2f} MB")


if __name__ == "__main__":
    for f in FILES:
        build(f)
