"""The front steering stage: Ackermann, rack travel per degree and steering ratio targets."""
import copy

import pytest

from fsaesusp import api, config, report
from fsaesusp.steeropt import (STEERING_VARS, apply_values, check_free, default_bounds, optimise_steering,
                               resolve_targets, steering_values)

QUIET = dict(quick=True, progress=lambda *a: None)


@pytest.fixture
def cfg(example_design):
    return config.load(example_design)


def test_ackermann_target_with_the_default_bounds(cfg):
    # the example sits at 51 % and 0.887 mm/deg: ask for 60 % at the same rack travel per degree
    v0 = steering_values(cfg)
    b = default_bounds(v0)                     # outer tie rod +/-15 across and fore/aft; height and rack fixed
    r = optimise_steering(cfg, b, dict(ackermann=60.0, rack_per_deg=0.887), **QUIET)
    a, v = r["after"], r["values"]
    assert a["ackermann"] == pytest.approx(60.0, abs=1.0)
    assert a["rack_per_deg"] == pytest.approx(0.887, abs=0.01)
    assert a["clearance"]["tie rod"] >= r["before"]["clearance"]["tie rod"] - 0.25   # default: no worse
    assert v["tie_rod_down"] == v0["tie_rod_down"] and v["rack_x"] == v0["rack_x"]
    assert all(b[k][0] <= v[k] <= b[k][1] for k in STEERING_VARS)
    new = r["cfg"]
    assert new["front"]["knuckle"]["tie_rod"] == [v["tie_rod_in"], v["tie_rod_down"]]
    for key in ("lower_pivot", "upper_pivot", "lower_arm", "upper_arm"):
        assert new["front"][key] == cfg["front"][key]
    assert new["front"]["knuckle"]["upper_ball_joint"] == cfg["front"]["knuckle"]["upper_ball_joint"]
    assert new["rear"] == cfg["rear"]


def test_values_round_trip_and_rack_offset(cfg):
    v = dict(steering_values(cfg), tie_rod_in=52.0, tie_rod_x=-55.0, rack_x=-40.0)
    new = apply_values(cfg, v)
    assert new["front"]["three_d"]["tie_rod"]["inner_x_offset"] == pytest.approx(15.0)
    assert steering_values(new) == pytest.approx(v)
    fixed = copy.deepcopy(cfg)                 # a full inner point holds the rack where it is
    fixed["front"]["three_d"]["tie_rod"]["inner"] = {"x": -30.0, "y": 250.0, "z": 150.0}
    assert steering_values(apply_values(fixed, dict(v, rack_x=-10.0)))["rack_x"] == -30.0


def test_steering_ratio_uses_the_c_factor(cfg):
    with pytest.raises(ValueError, match="c_factor"):
        resolve_targets(cfg, dict(steering_ratio=4.5))
    cfg["front"]["three_d"]["steering"]["c_factor"] = 90.0
    assert resolve_targets(cfg, dict(steering_ratio=4.5))["rack_per_deg"] == pytest.approx(1.125)
    with pytest.raises(ValueError, match="not both"):
        resolve_targets(cfg, dict(steering_ratio=4.5, rack_per_deg=1.0))
    rows = report.steering_rows(dict(ackermann_percent=60, inside_at_ack=25, inside_lock=27, outside_lock=22,
                                     rack_for_lock=25, rack_mm_per_deg=1.125, steering_ratio=4.5, c_factor=90.0,
                                     camber_inside_lock=0, camber_outside_lock=-3, trail_inside_lock=20,
                                     trail_outside_lock=5, lift_inside_lock=1, lift_outside_lock=-1,
                                     arm_tie_rod_angles=[40, 70, 95]))
    assert ["Steering ratio (centre)", "4.50:1 (c-factor 90 mm/rev)"] in rows


def test_refused_before_any_search(cfg):
    with pytest.raises(ValueError, match="steering only"):
        api.optimize(cfg, steering_only=True, **QUIET)              # no steering target
    o = cfg["optimise"]["front"]
    o["targets"]["ackermann"] = 60.0
    o["lock"] = list(STEERING_VARS)
    with pytest.raises(ValueError, match="optimise.front: steering targets need"):
        api.optimize(cfg, **QUIET)                                  # caught before the pivot search
    with pytest.raises(ValueError, match="can move"):
        check_free({k: [1.0, 1.0] for k in STEERING_VARS})
    o["lock"] = []
    cfg["front"]["three_d"]["tie_rod"]["design"] = {"side": "rear", "arm_length": 50, "arm_angle": 15}
    with pytest.raises(ValueError, match="tie_rod.design"):
        api.optimize(cfg, steering_only=True, **QUIET)


def test_setup_lists_the_steering_variables(cfg):
    st = api.optimise_setup(cfg, "front")
    v = steering_values(cfg)
    assert all(st["values"][k] == v[k] for k in STEERING_VARS)
    assert st["bounds"]["tie_rod_in"] == [v["tie_rod_in"] - 15, v["tie_rod_in"] + 15]
    assert st["bounds"]["rack_x"] == [v["rack_x"]] * 2
    assert not set(STEERING_VARS) & set(api.optimise_setup(cfg, "rear")["values"])


def test_cli_steering_only(example_design, tmp_path, capsys):
    from fsaesusp.cli import main
    cfg = config.load(example_design)
    o = cfg["optimise"]["front"]
    o["targets"]["ackermann"] = 50.7                           # the example's own value: a short run
    d = tmp_path / "d.yaml"; config.save(cfg, d)
    out = tmp_path / "s.yaml"
    assert main(["optimize", str(d), "-o", str(out), "--steering-only", "--quick", "--lock", "front.tie_rod_x"]) == 0
    text = capsys.readouterr().out
    assert "front steering: nothing moved" in text and "new pivots" not in text   # already on target
    new = config.load(out)
    assert new["optimise"]["front"]["lock"] == ["tie_rod_x"]
    assert new["front"]["three_d"]["tie_rod"]["x"] == cfg["front"]["three_d"]["tie_rod"]["x"]
    assert new["front"]["knuckle"]["tie_rod"] == cfg["front"]["knuckle"]["tie_rod"]
    assert new["front"]["lower_pivot"] == cfg["front"]["lower_pivot"] and new["rear"] == cfg["rear"]
