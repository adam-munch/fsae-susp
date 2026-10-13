import numpy as np
import pytest

from fsaesusp import api, config
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


def test_cli_compare(example_design, tmp_path, capsys):
    out = tmp_path / "cmp"
    assert main(["compare", str(example_design), str(example_design), "-o", str(out)]) == 0
    assert "Change" in capsys.readouterr().out
    assert all((out / f).exists() for f in ("front_view.png", "comparison.csv", "comparison.md"))
    assert main(["compare", str(example_design)]) == 2


def test_locks_keep_current_values(example_design, tmp_path):
    out = tmp_path / "opt.yaml"
    # rear: only lower_x is free; front: not optimised, but the lock given for it is saved
    assert main(["optimize", str(example_design), "-o", str(out), "--axle", "rear", "--quick",
                 "--lock", "lower_y,upper_x", "--lock", "rear.upper_y,rear.camber,front.lower_x"]) == 0
    before, after = config.load(example_design), config.load(out)
    assert after["rear"]["lower_pivot"][1] == pytest.approx(before["rear"]["lower_pivot"][1])
    assert after["rear"]["upper_pivot"] == pytest.approx(before["rear"]["upper_pivot"])
    assert 155 <= after["rear"]["lower_pivot"][0] <= 185           # free, within the file's bounds
    cam = [api.analyze(c)["rear"]["static"]["camber"] for c in (before, after)]
    assert cam[1] == pytest.approx(cam[0], abs=0.01)
    assert after["optimise"]["rear"]["lock"] == ["lower_y", "upper_x", "upper_y", "camber"]
    assert after["optimise"]["front"]["lock"] == ["lower_x"]
    assert after["front"] == before["front"]
    # --unlock frees what the file locks
    out2 = tmp_path / "opt2.yaml"
    assert main(["optimize", str(out), "-o", str(out2), "--axle", "rear", "--quick", "--lock", "lower_x",
                 "--unlock", "upper_y"]) == 0
    assert config.load(out2)["optimise"]["rear"]["lock"] == ["lower_x", "lower_y", "upper_x", "camber"]


def test_bad_lock_names(example_design, tmp_path, capsys):
    out = str(tmp_path / "x.yaml")
    assert main(["optimize", str(example_design), "-o", out, "--lock", "lower_z"]) == 2
    assert main(["optimize", str(example_design), "-o", out, "--lock", "middle.camber"]) == 2
    cfg = config.load(example_design); cfg["optimise"]["front"]["lock"] = ["upper"]
    with pytest.raises(ValueError, match="optimise.front.lock"):
        api.optimize(cfg, quick=True, progress=lambda *a: None)
    assert "unknown" in capsys.readouterr().err
