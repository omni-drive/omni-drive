#!/bin/sh
set -e

# Path to project root (compatible with sh / dash / bash)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -d "$SCRIPT_DIR/examples" ]; then
    ROOT_DIR="$SCRIPT_DIR"
elif [ -d "$SCRIPT_DIR/../examples" ]; then
    ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
else
    ROOT_DIR="$PWD"
fi

EXAMPLES_DIR="$ROOT_DIR/examples"

# Choose Python interpreter
if [ -n "$VIRTUAL_ENV" ] && [ -x "$VIRTUAL_ENV/bin/python" ]; then
    PYTHON="$VIRTUAL_ENV/bin/python"
elif [ -x "$ROOT_DIR/.venv/bin/python" ]; then
    PYTHON="$ROOT_DIR/.venv/bin/python"
elif command -v uv >/dev/null 2>&1; then
    PYTHON="uv run python"
else
    PYTHON="python3"
fi

export PYTHONPATH="$ROOT_DIR:${PYTHONPATH:-}"

show_help() {
    echo "=========================================================="
    echo " OmniDrive - Example Runner"
    echo "=========================================================="
    echo "Usage:"
    echo "  ./run_example.sh <name> [args...]"
    echo "  sh ./run_example.sh <name> [args...]"
    echo ""
    echo "Available examples in examples/:"
    for f in "$EXAMPLES_DIR"/*.py; do
        [ -e "$f" ] || continue
        base="$(basename "$f" .py)"
        if [ -L "$f" ]; then
            target="$(basename "$(readlink "$f")" .py)"
            printf "  %-14s -> alias for %s\n" "$base" "$target"
        else
            printf "  %s\n" "$base"
        fi
    done
    echo "=========================================================="
}

if [ $# -lt 1 ]; then
    show_help
    exit 0
fi

TARGET="$1"
TARGET_NAME="${TARGET%.py}"
SCRIPT_PATH="$EXAMPLES_DIR/$TARGET_NAME.py"

if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Error: Example '$TARGET' not found in $EXAMPLES_DIR"
    echo ""
    show_help
    exit 1
fi

shift
exec $PYTHON "$SCRIPT_PATH" "$@"
