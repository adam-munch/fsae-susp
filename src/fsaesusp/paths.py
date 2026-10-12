"""Where files go: imported designs in <project>/designs, named after the VSusp project."""
from __future__ import annotations

import re
from pathlib import Path


def project_root(start: Path) -> Path:
    """Nearest folder above ``start`` that holds pyproject.toml, else the folder ``start`` is in."""
    for d in Path(start).resolve().parents:
        if (d / "pyproject.toml").is_file():
            return d
    return Path(start).resolve().parent


def designs_dir() -> Path:
    """<project>/designs. The project is found from the working folder, then from this package's
    location (editable install); failing both, ./designs."""
    for start in (Path.cwd() / "_", Path(__file__)):
        root = project_root(start)
        if (root / "pyproject.toml").is_file():
            return root / "designs"
    return Path.cwd() / "designs"


def design_name(project_name: str) -> str:
    """File-name stem from a VSusp project name: 'MR2 V2.1' -> 'mr2_v2_1'."""
    return re.sub(r"[^a-z0-9]+", "_", (project_name or "").lower()).strip("_") or "design"


def default_design_path(project_name: str) -> Path:
    return designs_dir() / f"{design_name(project_name)}.yaml"
