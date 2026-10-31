#!/bin/bash
set -e

# Ensure ~/.local/bin is in PATH for uv
export PATH="$HOME/.local/bin:$PATH"

SKIP_DART=false
for arg in "$@"; do
    case $arg in
        --skip-dart)
            SKIP_DART=true
            shift
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

PROTO_DIR="$PROJECT_ROOT/proto"
WS_PY_OUT="$PROJECT_ROOT/deploy/services/generated"
DART_OUT="$PROJECT_ROOT/remote_control/lib/generated"

if [ ! -d "$PROTO_DIR" ]; then
    echo "Error: proto directory not found at $PROTO_DIR"
    exit 1
fi

if [ ! -f "$PROTO_DIR/remote.proto" ]; then
    echo "Error: remote.proto not found at $PROTO_DIR"
    exit 1
fi

mkdir -p "$WS_PY_OUT"
touch "$WS_PY_OUT/__init__.py"

if [ "$SKIP_DART" = false ]; then
    mkdir -p "$DART_OUT"
fi

GRPC_PROTOS=$(uv run --with grpcio-tools python -c "import grpc_tools, os; print(os.path.join(os.path.dirname(grpc_tools.__file__), '_proto'))")

cd "$PROTO_DIR" || exit 1

echo "Generating target-specific protobuf files for remote.proto..."

# 1. Python for WS Server (services)
uv run --with grpcio-tools python -m grpc_tools.protoc \
    -I . \
    -I "$GRPC_PROTOS" \
    --python_out="$WS_PY_OUT" \
    --pyi_out="$WS_PY_OUT" \
    --grpc_python_out="$WS_PY_OUT" \
    "remote.proto"

# 2. Dart / Flutter for Remote Control
if [ "$SKIP_DART" = false ]; then
    DART_PLUGIN="$HOME/.pub-cache/bin/protoc-gen-dart"
    if [ ! -f "$DART_PLUGIN" ] && ! command -v protoc-gen-dart >/dev/null 2>&1; then
        echo "Warning: protoc-gen-dart not found. Run 'dart pub global activate protoc_plugin' or use --skip-dart."
    else
        [ -f "$DART_PLUGIN" ] || DART_PLUGIN="$(command -v protoc-gen-dart)"
        uv run --with grpcio-tools python -m grpc_tools.protoc \
            -I . \
            -I "$GRPC_PROTOS" \
            --plugin=protoc-gen-dart="$DART_PLUGIN" \
            --dart_out="$DART_OUT" \
            "remote.proto" \
            "$GRPC_PROTOS/google/protobuf/empty.proto"
        DART_SUCCESS=true
    fi
else
    echo "Skipping Dart generation (--skip-dart enabled)."
fi

cd "$PROJECT_ROOT"

echo "Success! Generated:"
if [ "$SKIP_DART" = false ] && [ "${DART_SUCCESS:-false}" = true ]; then
    echo "  remote.proto -> Python ($WS_PY_OUT), Dart ($DART_OUT)"
else
    echo "  remote.proto -> Python ($WS_PY_OUT) [Dart skipped]"
fi

