#!/bin/bash
# Build webapp: embed path_graph.json into index.html
# Run after waypoint data changes: bash webapp/build.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA="$SCRIPT_DIR/../data/path_graph.json"
HTML="$SCRIPT_DIR/index.html"

if [ ! -f "$DATA" ]; then
  echo "ERROR: $DATA not found"
  exit 1
fi

# Generate JS variable from JSON
JS_DATA="const EMBEDDED_PATH_GRAPH = $(python3 -c "
import json, sys
with open('$DATA') as f:
    d = json.load(f)
print(json.dumps(d, separators=(',',':')))
");"

# Replace placeholder in HTML
python3 -c "
import sys
html = open('$HTML').read()
old = '/* __WAYPOINT_DATA_PLACEHOLDER__ */'
if old not in html:
    # Already has data, replace between markers
    import re
    html = re.sub(r'const EMBEDDED_PATH_GRAPH = .*?;', '/* __WAYPOINT_DATA_PLACEHOLDER__ */', html, count=1)
new = sys.stdin.read()
html = html.replace(old, new)
open('$HTML', 'w').write(html)
" <<< "$JS_DATA"

WP_COUNT=$(python3 -c "import json; print(len(json.load(open('$DATA'))['waypoints']))")
echo "Built: $WP_COUNT waypoints embedded into index.html"
