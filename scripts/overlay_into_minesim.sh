#!/usr/bin/env bash
# Optional: overlay strategy files onto a separately obtained official mine_sim tree.
# Primary path: this repo already contains a patched mine_sim — just clone and run.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/overlay_into_minesim.sh /path/to/mine_sim_root

Most users do NOT need this. The repository root is already a runnable
mine_sim with strategy files under ./devkit/. Prefer:

  git clone https://github.com/cainiao33/MineSim-MPC-AD-Control.git
  cd MineSim-MPC-AD-Control
  pip install -r setup/requirements.txt
  python devkit/__main__.py --scenario Scenario-dapai_intersection_1_3_10.json --visualize none

Use this script only if you already keep an official mine_sim checkout and
want to copy overlay/devkit/* into it (first-run backups → .mpc_ad_overlay_backup/).
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OVERLAY="$ROOT/overlay/devkit"
TARGET="${1%/}"
DEVKIT="$TARGET/devkit"

if [[ ! -d "$OVERLAY" ]]; then
  echo "error: overlay missing at $OVERLAY" >&2
  exit 1
fi
if [[ ! -d "$DEVKIT" ]]; then
  echo "error: target devkit not found: $DEVKIT" >&2
  echo "pass the mine_sim root that contains planner.py under devkit/" >&2
  exit 1
fi
for must in planner.py predictor.py simple_control.py; do
  if [[ ! -f "$DEVKIT/$must" ]]; then
    echo "error: expected stock file missing: $DEVKIT/$must" >&2
    exit 1
  fi
done

BACKUP="$TARGET/.mpc_ad_overlay_backup"
mkdir -p "$BACKUP"
while IFS= read -r -d '' src; do
  rel="${src#"$OVERLAY"/}"
  dst="$DEVKIT/$rel"
  if [[ -f "$dst" && ! -f "$BACKUP/$rel" ]]; then
    mkdir -p "$(dirname "$BACKUP/$rel")"
    cp -a "$dst" "$BACKUP/$rel"
  fi
done < <(find "$OVERLAY" -type f -print0)

mkdir -p "$DEVKIT/centerlines"
cp -a "$OVERLAY"/. "$DEVKIT"/

echo "overlay applied -> $DEVKIT"
echo "stock backups (first run) -> $BACKUP"
echo "run from the target root, e.g.:"
echo "  python devkit/__main__.py --scenario Scenario-dapai_intersection_1_3_10.json --visualize none"
