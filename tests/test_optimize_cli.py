import numpy as np
import pytest

from fsaesusp import config
from fsaesusp.optimize import optimise_axle
from fsaesusp.cli import main


def test_optimiser_respects_bounds(example_design):
    P0 = config.axle_fv(config.load(example_design), "rear")
    b = dict(lower_x=[150, 160], lower_y=[27.5, 27.5], upper_x=[255, 255], upper_y=[163, 163])   # 1 free variable
    r = optimise_axle(P0, b, dict(rc_height=45.0), quick=True, progress=lambda *a: None)
    assert 150 <= r["pivots"]["lower_x"] <= 160
    assert r["pivots"]["upper_x"] == 255 and r["pivots"]["lower_y"] == pytest.approx(27.5)
    assert np.isfinite(r["cost"])
    assert r["after"]["rc"] == pytest.approx(45.0, abs=3.0)


def test_cli_link_and_analyze(example_design, tmp_path, capsys):
    assert main(["link", str(example_design)]) == 0
    assert capsys.readouterr().out.strip().startswith("https://www.vsusp.com/#0.8")
    assert main(["analyze", str(example_design), "-o", str(tmp_path)]) == 0
    assert (tmp_path / "front_view.png").exists() and (tmp_path / "report_front_view.md").exists()


def test_cli_import(original_link, tmp_path):
    f = tmp_path / "link.txt"; f.write_text(original_link, encoding="utf-8")
    out = tmp_path / "imported.yaml"
    assert main(["import-vsusp", str(f), "-o", str(out)]) == 0
    cfg = config.load(out)
    assert cfg["front"]["lower_arm"] == pytest.approx(351.5)
