import copy
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]

from fsaesusp import api, config
from fsaesusp.adjust import adjustments, rules_check
from fsaesusp.cli import main
from fsaesusp.springs import G, SpringLayout, corner_springs, ride_frequency

S = dict(rod=dict(arm="lower", at=0.9, above=10.0),
         rocker=dict(pivot=[0.0, 250.0, 300.0], axis=[1.0, 0.0, 0.0], rod=[0.0, 274.9, 337.5], damper=[0.0, 250.0, 362.0]),
         damper=dict(chassis=[0.0, 78.0, 362.0], length=[144.0, 201.0]), spring_rate=52.5)


@pytest.fixture(scope="module")
def cfg():
    return config.load(ROOT / "examples" / "fsae_aero_v2.yaml")


@pytest.fixture(scope="module")
def sp(cfg):
    return api.springs(cfg, progress=lambda *a: None)


def test_motion_ratio_matches_lever_ratio(cfg):
    """With the rod square to the rocker lever, MR = rod motion ratio x (damper lever / rod lever)."""
    c, _ = api.build_3d(cfg, "front")
    lay = SpringLayout(c, S, 0.0)
    P, C = lay.P0, lay.C
    # rocker point Q on the 45 mm lever circle (plane x = 0) where the rod P-Q is tangent:
    # (Q - C).(Q - P) = 0  <=>  (Q - C).(P - C) = r^2
    p = (P - C)[1:]; a = np.arctan2(p[1], p[0]) + np.arccos(45.0 / np.linalg.norm(p))
    Q = C + 45.0 * np.array([0.0, np.cos(a), np.sin(a)])
    spec = copy.deepcopy(S); spec["rocker"]["rod"] = list(Q)
    spec["rocker"]["damper"] = list(C + np.array([0.0, 0.0, 62.0]))
    spec["damper"]["chassis"] = list(C + np.array([0.0, -172.0, 62.0]))   # square to its lever too
    r = corner_springs(c, spec, 0.0, dict(rate=110.0))
    assert r["rod_angle"] == pytest.approx(90.0, abs=1e-6) and r["damper_angle"] == pytest.approx(90.0, abs=1e-6)
    # the rocker point moves in the rocker plane; the rod leans out of it by beta
    cos_beta = np.sqrt(1 - ((Q - P) @ [1.0, 0.0, 0.0] / np.linalg.norm(Q - P)) ** 2)
    assert r["mr"] == pytest.approx(r["rod_mr"] / cos_beta * 62.0 / 45.0, rel=1e-6)
    assert r["wheel_rate"] == pytest.approx(52.5 * r["mr"] ** 2)
    assert r["ride_rate"] == pytest.approx(1 / (1 / r["wheel_rate"] + 1 / 110.0))


def test_example_springs(sp):
    f, r = sp["axles"]["front"], sp["axles"]["rear"]
    assert f["mr"] == pytest.approx(0.920, abs=0.002) and r["mr"] == pytest.approx(0.911, abs=0.002)
    assert 0 < f["progression"] < 15
    # the damper reaches its limits exactly at the travel reported
    lay = f["layout"]
    assert lay._len_from_static(f["bump_available"]) == pytest.approx(144.0, abs=1e-3)
    assert lay._len_from_static(-f["droop_available"]) == pytest.approx(201.0, abs=1e-3)
    assert f["ride_frequency"] == pytest.approx(ride_frequency(f["ride_rate"], 280 * 0.47 / 2 - 12.0))
    assert f["roll_wheel_rate"] == pytest.approx(1 / (1 / (f["wheel_rate"] + 19.2) + 1 / 110.0))


def test_vehicle_balance_identities(cfg, sp):
    v = sp["vehicle"]; m, h = 280.0, cfg["vehicle"]["cg_height"]
    tracks = {a: api.analyze(cfg)[a]["static"]["track"] / 1000 for a in api.AXLES}
    # lateral load transfer x track, summed over the axles, is the whole overturning moment m g h
    moment = sum(v["load_transfer"][a]["total"] * tracks[a] for a in api.AXLES)
    assert moment == pytest.approx(m * G * h / 1000, rel=1e-9)
    assert v["roll_gradient"] == pytest.approx(
        v["sprung_mass"] * G * (v["sprung_cg_height"] - v["roll_axis_height"]) / 1000 / v["roll_stiffness"])
    # braking: front dive = per-wheel transfer less the anti-dive share, through the ride rate
    dW = m * G * 1.5 * h / cfg["vehicle"]["wheelbase"] / 2
    anti = api.check3d(cfg, progress=lambda *a: None, steering=False)["front"]["anti"]["anti_dive"]
    assert v["pitch"]["braking"]["travel"]["front"] == pytest.approx(
        dW * (1 - anti / 100) / sp["axles"]["front"]["ride_rate"], rel=1e-6)
    assert v["assumed"] == []


def test_spring_on_one_axle_and_errors(cfg):
    c = copy.deepcopy(cfg); del c["rear"]["spring"]
    one = api.springs(c, progress=lambda *a: None)
    assert "vehicle" not in one and set(one["axles"]) == {"front"}
    c["front"]["spring"]["rocker"].pop("pivot")
    with pytest.raises(ValueError, match="spring.rocker.pivot"):
        api.springs(c, progress=lambda *a: None)
    del c["front"]["spring"]
    assert api.springs(c, progress=lambda *a: None) is None


def test_adjustments(cfg, sp):
    corners = {a: api.build_3d(cfg, a)[0] for a in api.AXLES}
    adj = adjustments(cfg, {a: config.axle_fv(cfg, a) for a in api.AXLES}, corners, sp)
    f = adj["front"]
    # a shim under the upper ball joint tilts the top of the wheel out: about 1 mm over the kingpin length
    kp = np.linalg.norm(corners["front"].hp["UBJ"] - corners["front"].hp["LBJ"])
    assert f["shim"]["camber"] == pytest.approx(np.degrees(1 / kp), rel=0.1)
    assert f["toe_per_turn"] == pytest.approx(f["toe_per_mm"] * 25.4 / 24)
    assert f["ride_per_perch_mm"] == pytest.approx(1 / sp["axles"]["front"]["mr"])
    assert f["ride_per_rod_mm"] == pytest.approx(1 / sp["axles"]["front"]["rod_mr"], rel=0.03)
    assert f["ride"]["rc"] == pytest.approx(-api.analyze(cfg)["front"]["metrics"]["rc_heave_slope"])


def test_rules(cfg, sp):
    rows = {r[0]: r for r in rules_check(cfg, sp)}
    assert rows["Wheelbase (mm)"][3] == "pass" and rows["Front jounce travel (mm)"][3] == "pass"
    c = copy.deepcopy(cfg); c["vehicle"]["wheelbase"] = 1500.0
    c["rules"] = {"track_ratio_min": 0.99, "travel_min": 32.0}
    rows = {r[0]: r for r in rules_check(c, sp)}
    assert rows["Wheelbase (mm)"][3] == "FAIL"
    assert rows["Front jounce travel (mm)"][3] == "FAIL" and rows["Front rebound travel (mm)"][3] == "FAIL"
    c["rules"] = {"travel_min": 30.0}
    assert {r[3] for r in rules_check(c, sp) if "travel" in r[0]} == {"pass"}
    assert "Smaller track / larger track" in rows
    c["rules"] = {"wheelbase": 1}
    with pytest.raises(ValueError, match="rules: unknown"):
        rules_check(c)


def test_cli_springs(example_design, tmp_path, capsys):
    out = tmp_path / "sp"
    assert main(["springs", str(example_design), "-o", str(out)]) == 0
    text = capsys.readouterr().out
    assert "Motion ratio" in text and "Roll gradient" in text and "Lateral load transfer distribution" in text
    assert (out / "springs.png").exists() and (out / "springs.md").exists()
