"""The front-view solver must reproduce VSusp's own numbers.

Reference values were read from vsusp.com's solver (v0.8) for the example link.
VSusp places the lower ball joint at whole-mm heights (99 mm front, 101 mm rear);
imposing the same heights must give identical geometry.
"""
import numpy as np
import pytest

from fsaesusp import vsusp
from fsaesusp.geometry2d import Corner2D, axle_state, line_intersect

REF = {
    "front": dict(lbj_y=99, L=(558.9861, 99.0), U=(534.9462, 293.7437), H=(583.8422, 202.7372),
                  W=(558.4477, 202.2110), T=(562.6576, -0.9454), O=(546.8129, 151.1590), camber=-1.18715,
                  ic=(-309, 52)),
    "rear": dict(lbj_y=101, L=(539.9658, 101.0), U=(523.4076, 291.0707), H=(564.1192, 204.4014),
                 W=(538.7201, 204.1933), T=(540.3853, 1.0001), O=(534.3661, 174.1566), camber=-0.46954,
                 ic=(-510, 87)),
}


@pytest.mark.parametrize("axle", ["front", "rear"])
def test_matches_vsusp_at_its_lbj_height(original_link, axle):
    P = vsusp.parse_link(original_link)[axle]
    c = Corner2D(P)
    p = c.solve_lbj_y(REF[axle]["lbj_y"])
    for k in ("L", "U", "H", "W", "T", "O"):
        assert np.allclose(p[k], REF[axle][k], atol=2e-4), k
    assert p["camber"] == pytest.approx(REF[axle]["camber"], abs=1e-4)
    ic = line_intersect(p["B"], p["U"], p["A"], p["L"])
    assert np.allclose(np.round(ic), [abs(REF[axle]["ic"][0]) * -1, REF[axle]["ic"][1]], atol=1.0)


def test_continuous_static_solution(original_link):
    d = vsusp.parse_link(original_link)
    f = axle_state(Corner2D(d["front"])); r = axle_state(Corner2D(d["rear"]))
    assert f["cam_r"] == pytest.approx(-1.2494, abs=1e-3)
    assert f["rc"][1] == pytest.approx(32.88, abs=0.02)
    assert r["rc"][1] == pytest.approx(45.33, abs=0.02)
    assert f["track"] == pytest.approx(1125.43, abs=0.02)
    assert abs(f["r"]["T"][1]) < 1e-8          # tread centre exactly on the ground
