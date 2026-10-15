"""Ball joints as optimiser variables, the KPI target and the scrub term."""
import pytest

from fsaesusp import api, config
from fsaesusp.analysis2d import outside_camber
from fsaesusp.optimize import optimise_axle

QUIET = dict(quick=True, progress=lambda *a: None)


def _fixed_pivots(P):
    return dict(lower_x=[P["cx_l"]] * 2, lower_y=[P["by_l"]] * 2, upper_x=[P["cx_u"]] * 2, upper_y=[P["by_u"]] * 2)


@pytest.fixture
def front(example_design):
    return config.axle_fv(config.load(example_design), "front")


def test_kpi_target_with_both_ball_joints_free(front):
    b = dict(_fixed_pivots(front), upper_bj_x=[40, 55], lower_bj_x=[20, 40])
    r = optimise_axle(front, b, dict(kpi=4.0), **QUIET)
    P = r["params"]
    assert r["after"]["kpi"] == pytest.approx(4.0, abs=0.01)
    assert r["kpi_joint"] == "upper_bj_x"                      # the upper joint follows the camber
    assert 40 - 0.005 <= P["hux"] <= 55 + 0.005 and 20 <= P["hlx"] <= 40
    assert P["hlx"] * 2 == round(P["hlx"] * 2)                # the searched joint is on the 0.5 mm grid
    assert (P["cx_l"], P["by_l"], P["cx_u"], P["by_u"]) == (front["cx_l"], front["by_l"], front["cx_u"], front["by_u"])
    assert (P["huy"], P["hly"]) == (front["huy"], front["hly"])   # heights on the upright are not touched
    # static camber still puts the outside wheel on its target with the joint following it
    avg = 0.5 * (outside_camber(P, 1.5, 0.0) + outside_camber(P, 1.8, 10.0))
    assert avg == pytest.approx(-1.5, abs=0.01)
    assert {"scrub", "outside_camber"} <= set(r["terms"])


def test_locked_upper_ball_joint_moves_the_lower_one(example_design, front):
    cfg = config.load(example_design)
    o = cfg["optimise"]["front"]
    o["bounds"] = dict(_fixed_pivots(front), upper_bj_x=[40, 55], lower_bj_x=[15, 45])
    o["targets"] = dict(o.get("targets") or {}, kpi=4.0)
    o["lock"] = ["upper_bj_x"]
    new, res = api.optimize(cfg, axles=("front",), **QUIET)
    assert res["front"]["kpi_joint"] == "lower_bj_x"
    assert new["front"]["knuckle"]["upper_ball_joint"][0] == pytest.approx(front["hux"])
    assert new["front"]["knuckle"]["lower_ball_joint"][0] != pytest.approx(front["hlx"])
    assert api.analyze(new)["front"]["static"]["kpi"] == pytest.approx(4.0, abs=0.01)
    assert new["rear"] == cfg["rear"]


def test_joint_at_its_limit_hands_the_kpi_to_the_other(front):
    # KPI 0 with negative camber puts the upper joint ~5 mm outboard of the lower one: with the lower joint
    # held near 20 mm the upper one would sit at ~15 mm, so it stops at its 40 mm limit and the lower one
    # is moved to hold the KPI, even though that is outside the lower joint's range (with a note).
    notes = []
    b = dict(_fixed_pivots(front), upper_bj_x=[40, 60], lower_bj_x=[20, 21])
    r = optimise_axle(front, b, dict(kpi=0.0), quick=True, progress=notes.append)
    P = r["params"]
    assert P["hux"] == 40.0
    assert r["kpi_joint"] == "lower_bj_x" and P["hlx"] > 21
    assert r["after"]["kpi"] == pytest.approx(0.0, abs=0.01)
    assert any("outside its bounds" in n for n in notes)


def test_kpi_target_needs_a_free_ball_joint(example_design, front):
    with pytest.raises(ValueError, match="ball joint"):
        optimise_axle(front, _fixed_pivots(front), dict(kpi=0.0), **QUIET)
    cfg = config.load(example_design)
    cfg["optimise"]["front"]["targets"]["kpi"] = 0.0          # no ball-joint bounds in the example
    with pytest.raises(ValueError, match="optimise.front"):
        api.optimize(cfg, **QUIET)                            # caught before either axle is searched


def test_setup_reports_ball_joints(example_design, front):
    st = api.optimise_setup(config.load(example_design), "front")
    assert st["values"]["upper_bj_x"] == front["hux"] and st["values"]["lower_bj_x"] == front["hlx"]
    assert st["bounds"]["upper_bj_x"] == [front["hux"]] * 2      # fixed unless the design file frees it
    assert st["targets"]["rc_height"] == pytest.approx(30.0)     # the example's own target
