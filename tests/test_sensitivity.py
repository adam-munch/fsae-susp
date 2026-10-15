import numpy as np
import pytest

from fsaesusp import api, config, sensitivity as sensmod
from fsaesusp.cli import main
from fsaesusp.layout3d import corner_from_2d
from fsaesusp.sensitivity import corner_sensitivity, kinematic_metrics, tolerances


def test_3d_roll_centre_matches_front_view(example_design):
    cfg = config.load(example_design); res2d = api.analyze(cfg)
    for axle in api.AXLES:
        c = corner_from_2d(config.axle_fv(cfg, axle), caster=0.0, slope_l=0.0, slope_u=0.0, otr_x=-60.0)
        m = kinematic_metrics(c, axle == "front", cfg["vehicle"])
        assert m["rc_height"] == pytest.approx(res2d[axle]["static"]["rc_height"], abs=1e-3)
        assert m["rc_heave_slope"] == pytest.approx(res2d[axle]["metrics"]["rc_heave_slope"], abs=1e-4)
        assert m["camber_gain"] == pytest.approx(res2d[axle]["metrics"]["camber_gain"], abs=1e-6)


def test_sensitivity_known_effects(example_design):
    cfg = config.load(example_design)
    c, _ = api.build_3d(cfg, "front")
    track = api.analyze(cfg)["front"]["static"]["track"]
    res = corner_sensitivity(c, True, cfg["vehicle"], tolerances(1.0, 0.5), track=track, points=("UBJ", "ITR"))
    rows = {(r["point"], r["axis"]): r for r in res["rows"]}
    assert len(rows) == 6
    # caster = atan((x_LBJ - x_UBJ) / (z_UBJ - z_LBJ)): d(caster)/d(x_UBJ) = -cos^2(caster) / kingpin height
    dz = c.hp["UBJ"][2] - c.hp["LBJ"][2]
    want = -np.degrees(np.cos(np.radians(5.0)) ** 2 / dz)
    assert rows["UBJ", "x"]["sens"]["caster"] == pytest.approx(want, rel=1e-3)
    assert rows["UBJ", "x"]["effect"]["caster"] == pytest.approx(0.5 * want, rel=1e-3)
    # links made to fit: a chassis point leaves the static alignment alone; as built it moves the toe
    assert abs(rows["ITR", "z"]["sens"]["caster"]) < 1e-6
    assert abs(rows["ITR", "y"]["sens"]["toe_static"]) > 0.5
    # the inner tie-rod height sets bump steer
    assert abs(rows["ITR", "z"]["sens"]["toe_bump"]) > 0.05
    assert max(rows, key=lambda k: abs(rows[k]["effect"]["toe_bump"])) == ("ITR", "z")
    st = res["stack"]["toe_bump"]
    assert st["rss"] <= st["worst"] and st["over"] == (st["rss"] > 0.05)
    assert sum(r["share"]["toe_bump"] for r in res["rows"]) == pytest.approx(1.0)


def test_cli_sensitivity(example_design, tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(sensmod, "POINTS", ("LBJ",))            # keep the test quick
    out = tmp_path / "sens"
    assert main(["sensitivity", str(example_design), "--axle", "rear", "--tol", "0.5", "-o", str(out)]) == 0
    text = capsys.readouterr().out
    assert "Rear corner" in text and "±0.5 mm (chassis)" in text and "Roll-centre height" in text
    assert all((out / f).exists() for f in ("sensitivity.csv", "sensitivity.png", "sensitivity.md"))
    lines = (out / "sensitivity.csv").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 4 and lines[1].startswith("rear,LBJ,x,0.5")
    assert main(["sensitivity", str(example_design), "--tol", "0"]) == 2
    cfg = config.load(example_design); cfg["sensitivity"] = {"allow": {"bump": 0.1}}
    with pytest.raises(ValueError, match="sensitivity.allow"):
        api.sensitivity(cfg, progress=lambda *a: None)
