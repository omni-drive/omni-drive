import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("grpc_tools")

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "gen_proto.py"
PACKAGE_DIR = SCRIPT.parent.parent / "omni_drive"


def _load_generator():
    spec = importlib.util.spec_from_file_location("gen_proto", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_committed_robot_pb2_matches_schema(tmp_path):
    _load_generator().generate(tmp_path)

    committed = (PACKAGE_DIR / "generated" / "robot_pb2.py").read_text(encoding="utf-8")
    regenerated = (tmp_path / "robot_pb2.py").read_text(encoding="utf-8")
    assert regenerated == committed
