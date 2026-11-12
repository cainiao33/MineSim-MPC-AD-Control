#!/usr/bin/env bash
# Overlay MineSim-MPC-AD-Control strategy files onto an official mine_sim tree.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/overlay_into_minesim.sh /path/to/mine_sim_root

Copies overlay/devkit/* into <mine_sim_root>/devkit/, preserving relative paths.
Does not touch inputs/, datasets/, or other official assets.

Example:
  bash scripts/overlay_into_minesim.sh ~/mine_sim_V1.1_local
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

# Backup stock files we overwrite (once).
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
echo "verify: ls $DEVKIT/centerlines/scen_config.json"
echo "verify: ls $DEVKIT/sas_core.py $DEVKIT/stpp_hook.py"
echo "run a public scenario from the mine_sim root, e.g.:"
echo "  python devkit/__main__.py --scenario Scenario-dapai_intersection_1_3_10.json --visualize none"
