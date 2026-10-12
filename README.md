# fsae-susp

Command-line tools for FSAE double-wishbone suspension design:

* **Front view, compatible with VSusp.** Import a [vsusp.com](https://www.vsusp.com) link, analyse roll centres,
  camber curves and migration in heave and roll, and export a VSusp link again. The solver matches VSusp
  exactly at the same ball-joint position. It solves continuously, while VSusp moves in 1 mm steps, so
  slopes and migration numbers are smooth.
* **Pivot optimiser.** Searches chassis pickup positions within your packaging limits. It hits roll-centre
  height, heave and roll migration, swing-arm length and arm-angle targets, then picks the static camber so
  the outside tyre lands on a camber target.
* **3D check.** Adds caster and trail, side-view pivot-axis tilt (anti-dive, anti-squat, anti-lift), the
  steering arm and rack, and the rear toe link. It places the inner tie-rod and toe-link points for zero
  bump steer in 3D. It then reports bump and roll steer, Ackermann, steering lock, camber in steer, and
  arm, tie-rod and rim clearance at full lock.
* **Plots and report.** Writes PNG charts, a Markdown report and a 3D hardpoint CSV for CAD.

## Install

You need Python 3.9 or newer.

**Windows (PowerShell)**

```powershell
cd $HOME\Projects\fsae-susp
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1      # or .venv\Scripts\activate.bat in cmd
pip install -e ".[test]"
fsae-susp --version
```

**macOS / Linux**

```bash
cd ~/Projects/fsae-susp
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[test]"
```

`python -m fsaesusp ...` works as well as `fsae-susp ...`.

## Quick start

```bash
# 1. Turn a VSusp link into a design file.
#    The link can be in quotes, or in a text file (easier on Windows, where % can trip up cmd).
#    It is saved as designs/<project name>.yaml; -o picks another name.
fsae-susp import-vsusp examples/original_vsusp_link.txt -o designs/mycar.yaml

# 2. Front-view analysis: table on screen, plots and report in results/mycar/
fsae-susp analyze designs/mycar.yaml -o results/mycar

# 3. Optimise pickups. Set bounds and targets under `optimise:` in the design file first.
#    --quick takes about 1 min per axle; drop it for the full search (about 3-4 min per axle).
fsae-susp optimize designs/mycar.yaml -o designs/mycar_opt.yaml --quick --plot

# 4. 3D check (edit the `three_d:` blocks first: caster, anti %, pivot spacing, tie rod)
fsae-susp check3d designs/mycar_opt.yaml -o results3d/mycar_opt

# 5. Everything in one go, plus a Markdown report
fsae-susp report designs/mycar_opt.yaml -o report/mycar_opt

# Get the VSusp link back to view the design in the browser
fsae-susp link designs/mycar_opt.yaml
```

`examples/fsae_aero_v2.yaml` is a complete design (the 3D-checked aero redesign) with every field commented.

## GUI

`fsae-susp-gui` (or `python -m fsaesusp.gui`) opens a small window. Pick a design file or paste a VSusp
link, then click Analyze, Optimise, 3D check, Full report or VSusp link. The GUI runs the same commands as the
command line. Their output appears in the Output tab, and the PNGs they write appear in the Plots tab.
The Plots tab shows all of a design's plots together: the front-view plot from Analyze, and the 3D plots
from 3D check, each from the latest run. Click a plot, or switch to One, to see it on its own.

The Projects list on the left shows every design in `designs/`, newest first, with its VSusp project name and
which results exist. Double-click a design to open it and see its latest plots. To compare designs, Ctrl+click
2 to 4 of them and click Compare. The Compare tab then shows their front-view metrics side by side, with rows
that differ shaded and the change from the older design to the newer one. The Plots tab shows them overlaid.
Comparisons are saved in `results/compare/<a>_vs_<b>/`. Right-click a design to analyse, check or report on
it directly, or to show it in its folder.

Shortcuts: Ctrl+O picks a design file, F5 reloads the designs and plots, Esc stops the running command, and
the Left and Right arrow keys step through the plots in the Plots tab. Hover over a button to see what it does.

Imported links are saved as `designs/<name>.yaml`, as on the command line. The name defaults to the VSusp
project name, and you can change it before saving.

The GUI chooses the output folders, using the design file's name for the subfolder. For `sastha.yaml`, Analyze
writes to `results/sastha/`, 3D check to `results3d/sastha/` and Full report to `report/sastha/`. These folders
are in the project root. Optimise saves the new design next to the original (`sastha_opt.yaml` by default) and
its plots in `sastha_opt_plots/`.

## Commands

| Command | What it does |
|---|---|
| `import-vsusp LINK [-o design.yaml]` | New design file from a VSusp link (or a text file holding it). Without `-o` it is saved as `designs/<VSusp project name>.yaml`. 3D and vehicle values start at defaults. |
| `link design.yaml` | Print the VSusp link for the design. |
| `analyze design.yaml [-o DIR] [--compare other.yaml]` | Front-view table and flags. With `-o`: `front_view.png` and `report_front_view.md`. `--compare` overlays a second design. |
| `compare a.yaml b.yaml [c.yaml d.yaml] [-o DIR]` | Front-view metrics of 2 to 4 designs side by side, with the change from the first to the second when there are two. With `-o`: an overlay `front_view.png`, `comparison.csv` and `comparison.md`. |
| `optimize design.yaml -o new.yaml [--axle front/rear/both] [--quick] [--workers N] [--plot]` | Optimise inner pivots and write a new design file. `--workers -1` uses all cores. |
| `check3d design.yaml [-o DIR] [--no-steering] [--sync]` | 3D tables and flags. With `-o`: `hardpoints_3d.csv`, `check3d.png`, `layout_3d.png`, `report_3d.md`. `--sync` writes the 3D tie-rod points back into the design file. |
| `report design.yaml -o DIR` | Front view and 3D check, all plots, CSV and `report.md`. |

## The design file

YAML (or JSON). See `examples/fsae_aero_v2.yaml`. Units are mm and degrees.

**Front-view fields use VSusp's conventions**, so files round-trip with vsusp.com:

* `lower_pivot` / `upper_pivot`: `[x from frame centre, y above frame bottom]`. `ride_height` is the frame
  bottom above ground.
* `knuckle`: measured from the **hub face**, as `[inboard, up]` for the upper ball joint and
  `[inboard, down]` for the lower ball joint and tie rod.
* `wheel.offset`: how far the hub face sits outboard of the wheel centreline. If you measured your knuckle
  from the wheel centreline instead, set `offset: 0`. This moves scrub and track by about the offset.
* `tyre.static_deflection`: the loaded radius is `diameter/2 - static_deflection`.

**3D fields** (`front.three_d`, `rear.three_d`) use x forward (front axle at 0), y to the left and z up:

* `caster`, `kingpin_offset_x`: these move the ball joints fore and aft. The kingpin passes
  `kingpin_offset_x` ahead of the wheel centre at hub height.
* `anti`: % anti-dive (front) or % anti-squat (rear). Both pivot axes of the axle are tilted in parallel
  in side view, so caster does not change with travel. `pivot_axis_slope` (dz/dx) overrides it.
* `lower_pivot_x`, `upper_pivot_x`: front and rear pivot positions relative to the ball joint's x.
* `tie_rod`: the outer tie rod (front) or toe link (rear) sits at `x` relative to the axle. Its y and z come
  from `knuckle.tie_rod`. The inner point has three options:
  * `inner: auto` places it for zero bump steer in 3D. The lateral position comes from the front-view
    zero-bump-steer point, and the height is solved in 3D to minimise toe change over ±25 mm.
  * `inner: {y: ...}` fixes the rack-end lateral position and still solves the height.
  * `inner: {x, y, z}` gives the full point.

  To design the arm instead, use `design: {side: rear|front, arm_length, arm_angle, below_hub}`.
  Then run `check3d --sync` to copy the resulting points back into the file.
* `steering.lock_inside`: the inside-wheel lock to check. A 9 m FSAE hairpin needs about 27°.
* `vehicle`: wheelbase, CG height, front brake bias and `inboard_drive`. These feed the anti percentages.
* `wheel.lip_inboard`, `lip_radius`, `link_radius`, `node_inboard_of_lip`: the clearance model (see below).
  **Measure your own wheel.**

`optimise:` holds the optimiser settings:

* `camber`: the outside-wheel target and the low- and high-speed (roll, heave) cases used to pick static
  camber.
* `front/rear.targets`: for example `rc_height`, `fvsa_range`, `lca_incl_range`, `rc_heave_slope_max`,
  `rc_lateral_tol`, `lower_arm_min`, `upper_arm_min`. The full list is `DEFAULT_TARGETS` in
  `src/fsaesusp/optimize.py`.
* `front/rear.bounds`: `lower_x`, `lower_y`, `upper_x`, `upper_y` as `[lo, hi]` in VSusp coordinates.
  `lo == hi` fixes that coordinate. Without bounds, the search stays within ±30 mm of the current pivots.

## How it works

* **Front view** (`geometry2d.py`): a rigid knuckle on two arms, with the tread centre held on the ground.
  This is VSusp's model, solved by root-finding on the lower-arm angle. Roll rotates the body about the
  frame-bottom centre with both tyres on the ground, as in VSusp.
* **Optimiser** (`optimize.py`): differential evolution, then a bounded Nelder-Mead polish, over the four
  pivot coordinates. Arm lengths are derived so track and static camber hold. The cost is a sum of squared
  normalised misses: RC height, RC heave slope above a threshold, RC lateral migration, swing-arm band, RC
  at the heave case, half-track change and packaging (arm lengths, arm angles, frame depth). Afterwards,
  static camber is chosen so the outside wheel averages `outside_target` over the two cases. Pivots are
  rounded to 0.5 mm and arm lengths to 0.1 mm.
* **3D** (`geometry3d.py`, `layout3d.py`): the upright pose has 6 degrees of freedom. Least-squares solves
  four ball-joint-to-pivot distances, the tie-rod length and the contact-patch height. Anti percentages
  come from virtual displacements:
  * outboard brakes use the contact patch;
  * inboard drive uses the wheel centre.
* **Zero bump steer**: the inner point's height is found by a 1-D search that minimises the RMS toe change
  over ±25 mm of travel, with x and y fixed. Moving the point along the tie rod changes bump steer only
  slightly, so the lateral position is a packaging choice.
* **Steering** (`steering.py`): left and right corners, with rack travel tracked by continuation.
  * Ackermann % = (inside − outside) / (inside − ideal outside) at `ackermann_at`.
  * It also reports lock, rack travel per degree, camber, trail and ball-joint lift at lock, and the
    arm-to-tie-rod angle as a lock-up check.
* **Clearance** (`clearance.py`) is a screening model:
  * the rim is an open barrel with its inboard lip `lip_inboard` from the centreline and inner radius
    `lip_radius`;
  * the tyre is a cylinder;
  * links are tubes of radius `link_radius`;
  * each A-arm's legs meet at a node `node_inboard_of_lip` past the lip, with one stem running into the
    wheel.
  It is checked over the steering range at heave −15, 0 and +15 mm (front), and over ±30 mm travel (rear).
  **Confirm in CAD with your wheel, brake rotor, caliper and upright.**

## Validation

`pytest` runs the checks below (about 20 s):

* The front-view solver reproduces VSusp's own geometry to 0.0002 mm at the same lower-ball-joint height,
  using values read from vsusp.com for `examples/original_vsusp_link.txt`.
* VSusp links round-trip, and so do design files.
* With zero caster and no pivot-axis tilt, the 3D solver reproduces the 2D camber and contact-patch path.
* The example design's 3D results stay as checked: 20 % anti-dive, 25 % anti-squat, 51 % Ackermann, and
  zero bump steer.
* The optimiser respects its bounds.

## Limitations

* Double wishbone only. Other VSusp suspension types are rejected on import.
* Rigid links. There is no compliance, tyre lateral deflection or pneumatic trail.
* Symmetric car: the right side is the mirror of the left.
* Roll is a pure rotation about the frame-bottom centre with both tyres on the ground, and heave is
  body-fixed with the ground raised. This matches VSusp.
* There are no springs or rockers yet, so no motion ratios.
* Clearance numbers depend on the simple wheel model above.

## Python API

```python
from fsaesusp import config, api
cfg = config.load("examples/fsae_aero_v2.yaml")
res = api.analyze(cfg)            # front view: res["front"]["metrics"]["rc_heave_slope"], ...
res3d = api.check3d(cfg)          # 3D: res3d["steering"]["ackermann_percent"], ...
print(config.to_vsusp_link(cfg))
```
