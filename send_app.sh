#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

ROBOT_HOST="${1:-omni@omnidrive.local}"
ROBOT_PATH="${2:-/home/omni/omni-drive-public}"

echo "=================================================="
echo "   OmniDrive - Build & Deploy Remote Control"
echo "=================================================="
echo "Target robot: $ROBOT_HOST"
echo "Target path:  $ROBOT_PATH"
echo ""

# ==========================================
# 1. Generate Protobuf definitions
# ==========================================
echo "[1/3] Generating Protobuf definitions..."
if [ -f "$PROJECT_ROOT/gen_proto.sh" ]; then
  "$PROJECT_ROOT/gen_proto.sh"
else
  echo "Error: gen_proto.sh not found at $PROJECT_ROOT/gen_proto.sh"
  exit 1
fi

# ==========================================
# 2. Build Flutter Web application
# ==========================================
echo ""
echo "[2/3] Building Flutter Web client (offline / CanvasKit local)..."
if ! command -v flutter >/dev/null 2>&1; then
  echo "Error: 'flutter' CLI is not found in PATH."
  exit 1
fi

(
  cd "$PROJECT_ROOT/remote_control"
  flutter build web --release --base-href /joystick/ --no-web-resources-cdn
)

# ==========================================
# 3. Synchronize build artifacts to robot
# ==========================================
echo ""
echo "[3/3] Transferring build artifacts to $ROBOT_HOST..."

# Ensure target directories exist on robot
ssh "$ROBOT_HOST" "mkdir -p '$ROBOT_PATH/deploy/services/generated' '$ROBOT_PATH/remote_control/build/web'"

if command -v rsync >/dev/null 2>&1; then
  echo "Using rsync for incremental synchronization..."
  rsync -avz "$PROJECT_ROOT/deploy/services/generated/" "$ROBOT_HOST:$ROBOT_PATH/deploy/services/generated/"
  rsync -avz --delete "$PROJECT_ROOT/remote_control/build/web/" "$ROBOT_HOST:$ROBOT_PATH/remote_control/build/web/"
else
  echo "rsync not found locally, falling back to scp..."
  scp -r "$PROJECT_ROOT/deploy/services/generated/"* "$ROBOT_HOST:$ROBOT_PATH/deploy/services/generated/"
  scp -r "$PROJECT_ROOT/remote_control/build/web/"* "$ROBOT_HOST:$ROBOT_PATH/remote_control/build/web/"
fi

echo ""
echo "=================================================="
echo "Deployment transfer completed successfully!"
echo ""
echo "To reload services on the robot, run:"
echo "  ssh $ROBOT_HOST 'cd $ROBOT_PATH/deploy && sudo ./install.sh'"
echo "=================================================="

