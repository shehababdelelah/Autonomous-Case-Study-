"""Export the ESP32 page as a standalone browser preview with simulated sensors.

Run: python tools/preview_webpage.py
Then open preview/webpage.html in any browser. No ESP32 or server is required.
"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src/WEBpage.cpp").read_text(encoding="utf-8-sig")
match = re.search(r'R"rawpage\((.*?)\)rawpage"', source, re.DOTALL)
if not match:
    raise SystemExit("Could not find the embedded webpage in src/WEBpage.cpp")

page = match.group(1).replace("%TITLE%", "CAS Box — UI preview")
page = page.replace("%MAXSPD%", "0.50")
simulation = (ROOT / "tools/webpage_preview_sim.js").read_text(encoding="utf-8")
page = page.replace("<script>", f"<script>\n{simulation}\n</script>\n<script>", 1)
banner = (
    '<div role="note" style="padding:12px 16px;background:#183652;'
    'color:#e7edf5;text-align:center;font:14px system-ui">'
    '<strong>LOCAL PREVIEW</strong> · Simulated sensor readings · '
    'Buttons only affect this preview; no robot connection</div>'
)
page = page.replace("<body>", f"<body>{banner}", 1)
output = ROOT / "preview/webpage.html"
output.parent.mkdir(exist_ok=True)
output.write_text(page, encoding="utf-8")
print(f"Preview created: {output}")
print("Open this HTML file in your browser. Rerun this script after editing WEBpage.cpp.")
