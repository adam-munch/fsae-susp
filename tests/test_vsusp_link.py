import pytest
from fsaesusp import vsusp, config


def test_parse_original(original_link):
    d = vsusp.parse_link(original_link)
    assert d["project_name"] == "suspension geometry latest"
    f = d["front"]
    assert f["cx_u"] == 260 and f["by_u"] == 155 and f["lower_len"] == 351.5
    assert f["tire_d"] == pytest.approx(406.4) and f["wheel_d"] == pytest.approx(254.0)
    assert d["rear"]["hly"] == pytest.approx(103.2)


def test_round_trip(original_link):
    d = vsusp.parse_link(original_link)
    link = vsusp.build_link(d["front"], d["rear"], "round trip")
    d2 = vsusp.parse_link(link)
    for axle in ("front", "rear"):
        for k, v in d[axle].items():
            assert d2[axle][k] == pytest.approx(v, abs=0.006), (axle, k)
    assert d2["project_name"] == "round trip"


def test_design_file_round_trip(example_design, tmp_path):
    cfg = config.load(example_design)
    out = tmp_path / "copy.yaml"
    config.save(cfg, out)
    cfg2 = config.load(out)
    for axle in ("front", "rear"):
        assert config.axle_fv(cfg2, axle) == pytest.approx(config.axle_fv(cfg, axle))
    d = vsusp.parse_link(config.to_vsusp_link(cfg))
    assert d["front"]["hly"] == pytest.approx(88.0)
