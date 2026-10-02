import sys
from pathlib import Path

from grpc_tools import protoc

PACKAGE_DIR = Path(__file__).resolve().parent.parent / "omni_drive"
PROTO_DIR = PACKAGE_DIR / "proto"
GENERATED_DIR = PACKAGE_DIR / "generated"


def generate(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    exit_code = protoc.main(
        ["grpc_tools.protoc", f"-I{PROTO_DIR}", f"--python_out={output_dir}", "robot.proto"]
    )
    if exit_code != 0:
        raise RuntimeError(f"protoc failed with exit code {exit_code}")


if __name__ == "__main__":
    generate(Path(sys.argv[1]) if len(sys.argv) > 1 else GENERATED_DIR)
