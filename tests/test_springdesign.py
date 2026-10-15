import copy
from pathlib import Path

import numpy as np
import pytest

from fsaesusp import api, config
from fsaesusp import springdesign as sdm
from fsaesusp.cli import main
from fsaesusp.springs import corner_springs, seg_dist

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def cfg():
    return config.load(ROOT / "examples" / "fsae_aero_v2.yaml")


@pytest.fixture(scope="module")
def front(cfg):
    c, _ = api.build_3d(cfg, "front")
    return c, sdm.setup(cfg, "front", c)


def test_seg_dist_matches_sampling():
    rng = np.random.default_rng(1)
    for _ in range(20):
        p1, q1, p2, q2 = rng.normal(size=(4, 3)) * 50
        t = np.linspace(0, 1, 801)
        A = p1 + t[:, None] * (q1 - p1); B = p2 + t[:, None] * (q2 - p2)
        brute = np.min(np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2))
        assert float(seg_dist(p1, q1, p2, q2)) == pytest.approx(brute, abs=0.2)
        assert float(seg_dist(p1, q1, p2, q2)) <= brute + 1e-9


def test_generated_layout(front):
    c, st = front
    v = sdm.construct(st)
    g = sdm.build(v, st["ctx"]); e = sdm.evaluate(g, st["ctx"])
    T = st["ctx"]["targets"]
    assert e["mr"] == pytest.approx(T["mr"], abs=0.002)                # damper lever sized for the target
    assert e["mr"] > 0 and e["rod_mr"] > 0                              # the damper compresses in bump
    # static length placed so bump and droop have the same margin, and the stroke is used exactly
    L = lambda z: np.interp(z, *(lambda d: (d["z"], d["L"]))(sdm.evaluate(g, st["ctx"], detail=True)))
    B, D = T["travel"]
    assert L(B + e["margin"]) == pytest.approx(st["ctx"]["damper"][0], abs=0.05)
    assert L(-(D + e["margin"])) == pytest.approx(st["ctx"]["damper"][1], abs=0.05)
    assert min(e["min_rod_angle"], e["min_damper_angle"]) > 65         # square levers stay near square


def test_fast_evaluation_matches_the_analysis(cfg, front):
    """The search's quick closed-form evaluation agrees with the full springs analysis of the block it writes."""
    c, st = front
    v = sdm.construct(st)
    J, terms, g, e = sdm.cost(v, st["ctx"], detail=True)
    spec = sdm.spring_block(dict(values=v, geometry=g, result=e), st, 52.5)
    tyre, wheel = config.axle_tyre_wheel(cfg, "front")
    a = corner_springs(c, spec, 0.0, tyre, wheel=wheel)
    B, D = st["ctx"]["targets"]["travel"]
    assert a["mr"] == pytest.approx(e["mr"], abs=0.003)
    assert a["bump_available"] == pytest.approx(B + e["margin"], abs=0.3)
    assert a["droop_available"] == pytest.approx(D + e["margin"], abs=0.3)
    assert a["min_rod_angle"] == pytest.approx(e["min_rod_angle"], abs=0.5)
    assert a["min_damper_angle"] == pytest.approx(e["min_damper_angle"], abs=0.5)
    assert max(abs(x) for x in a["rocker_range25"]) == pytest.approx(e["max_rotation"], abs=0.3)
    assert a["rod_gap_links"] == pytest.approx(e["rod_gap"], abs=0.3)
    assert a["progression"] == pytest.approx(e["progression"], abs=0.5)


def test_search_meets_targets(cfg, front):
    c, st = front
    r = sdm.optimise_axle(st, quick=True, progress=lambda *a: None)
    e, T = r["result"], st["ctx"]["targets"]
    assert r["cost"] < r["start"]["cost"]
    assert e["mr"] == pytest.approx(T["mr"], abs=0.003)
    assert e["margin"] >= T["travel_margin"] - 0.05
    assert min(e["min_rod_angle"], e["min_damper_angle"]) >= T["min_angle"] - 0.5
    assert e["rod_gap"] >= T["rod_clearance"] - 0.1 and e["rim_gap"] >= T["rim_clearance"]
    for k, (lo, hi) in st["bounds"].items():
        assert lo - 1e-9 <= r["values"][k] <= hi + 1e-9


def test_locks_hold_the_spring_blocks_values(cfg):
    c, _ = api.build_3d(cfg, "rear")
    st = sdm.setup(cfg, "rear", c, lock_extra=("pivot_x", "pivot_y", "pivot_z", "rod_lever"))
    cur = sdm.current_values(cfg["rear"]["spring"])
    r = sdm.optimise_axle(st, construct_only=True, progress=lambda *a: None)
    for k in ("pivot_x", "pivot_y", "pivot_z", "rod_lever"):
        assert r["values"][k] == pytest.approx(cur[k]) and k not in r["free"]
    bad = copy.deepcopy(cfg); bad["spring_design"]["rear"]["lock"] = ["rocker"]
    with pytest.raises(ValueError, match="lock: unknown"):
        sdm.setup(bad, "rear", c)
    nobounds = copy.deepcopy(cfg); del nobounds["rear"]["spring"]; nobounds["spring_design"]["rear"].pop("bounds")
    with pytest.raises(ValueError, match="bounds: give pivot_x"):
        sdm.setup(nobounds, "rear", c)


def test_rates():
    exact, chosen = sdm.spring_for_frequency(3.85, 53.8, 0.92, 110.0)
    k_ride = 53.8 * (2 * np.pi * 3.85) ** 2 / 1000
    assert exact == pytest.approx(k_ride * 110 / (110 - k_ride) / 0.92 ** 2)
    assert chosen / sdm.LB_IN == pytest.approx(300.0)                  # nearest 25 lb/in
    assert sdm.spring_for_frequency(3.85, 53.8, 0.92, 110.0, rates=[40.0, 55.0, 60.0])[1] == 55.0
    with pytest.raises(ValueError, match="tyre rate"):
        sdm.spring_for_frequency(12.0, 53.8, 0.92, 110.0)


def test_cli_design_springs(example_design, tmp_path, capsys):
    out = tmp_path / "sub" / "new.yaml"
    assert main(["design-springs", str(example_design), "-o", str(out), "--no-optimise"]) == 0
    text = capsys.readouterr().out
    assert "Front spring layout" in text and "anti-roll bar:" in text and "Generated" in text
    new = config.load(out)
    assert new["front"]["spring"]["damper"]["length"] == [144.0, 201.0]
    assert new["front"]["spring"]["spring_rate"] / sdm.LB_IN == pytest.approx(300.0, abs=0.5)
    sp = api.springs(new, progress=lambda *a: None)
    assert sp["vehicle"]["lltd_front"] == pytest.approx(0.50, abs=0.002)    # bar sized for the target
    assert main(["design-springs", str(example_design), "-o", str(out), "--lock", "front.rocker"]) == 2
