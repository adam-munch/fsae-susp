from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def original_link():
    return (ROOT / "examples" / "original_vsusp_link.txt").read_text(encoding="utf-8").strip()


@pytest.fixture
def example_design():
    return ROOT / "examples" / "fsae_aero_v2.yaml"
