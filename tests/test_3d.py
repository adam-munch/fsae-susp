import numpy as np
import pytest

from fsaesusp import config, api
from fsaesusp.geometry2d import Corner2D
from fsaesusp.layout3d import corner_from_2d


def test_3d_reduces_to_2d_without_caster_or_tilt(example_design):
    P = config.axle_fv(config.load(example_design), "front")
    c3 = corner_from_2d(P, caster=0.0, slope_l=0.0, slope_u=0.0, otr_x=-60.0)
    c2 = Corner2D(P)
    s0 = c3.solve(0)
    for z in (-25, -10, 10, 25):
        s3, p2 = c3.solve(z), c2.solve(0, z)
        assert s3["camber"] == pytest.approx(p2["camber"], abs=1e-6)
        assert s3["CP"][1] == pytest.approx(p2["T"][0], abs=1e-6)
        assert abs(s3["toe_in"]) < 0.01                       # front-view zero-bump-steer point
    c3.solve(0)


def test_check3d_example(example_design):
    cfg = config.load(example_design)
    res = api.check3d(cfg, progress=lambda *a: None)
    f, r, st = res["front"], res["rear"], res["steering"]
    assert f["static"]["caster"] == pytest.approx(5.0, abs=0.01)
    assert f["static"]["trail"] == pytest.approx(17.6, abs=0.2)
    assert f["anti"]["anti_dive"] == pytest.approx(20.0, abs=0.6)
    assert r["anti"]["anti_squat"] == pytest.approx(25.0, abs=0.6)
    assert max(abs(np.array(f["bump"]["toe_in"]))) < 0.01
    assert max(abs(np.array(r["bump"]["toe_in"]))) < 0.01
    assert st["ackermann_percent"] == pytest.approx(51, abs=2)
    assert st["inside_lock"] == pytest.approx(27.0, abs=0.01)
    assert st["rack_for_lock"] == pytest.approx(21.3, abs=0.3)
    assert f["clearance_worst"]["lower arm"]["clearance"] == pytest.approx(3.6, abs=0.5)
    assert f["clearance_worst"]["tie rod"]["clearance"] > 3.0
