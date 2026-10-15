# fsae-susp

Command-line tools for FSAE double-wishbone suspension design:

* **Front view, compatible with VSusp.** Import a [vsusp.com](https://www.vsusp.com) link, analyse roll centres,
  camber curves and migration in heave and roll, and export a VSusp link again. The solver matches VSusp
  exactly at the same ball-joint position. It solves continuously, while VSusp moves in 1 mm steps, so
  slopes and migration numbers are smooth.
* **Pivot optimiser.** Searches chassis pickup positions within your packaging limits. It hits roll-centre
  height, heave and roll migration, swing-arm length and arm-angle targets, then picks the static camber so
  the outside tyre lands on a camber target. Give the ball joints a range on the upright and it moves them
  too, keeping scrub radius low. A KPI target is then held exactly. With an Ackermann or steering-ratio
  target it also places the front outer tie rod and rack line, keeping the tie rod clear of the rim.
* **3D check.** Adds caster and trail, side-view pivot-axis tilt (anti-dive, anti-squat, anti-lift), the
  steering arm and rack, and the rear toe link. It places the inner tie-rod and toe-link points for zero
  bump steer in 3D. It then reports bump and roll steer, Ackermann, steering lock, camber in steer, and
  arm, tie-rod and rim clearance at full lock.
* **Springs.** A pushrod or pullrod, rocker and damper on each axle give the motion ratio over travel, the
  damper stroke used, the wheel, ride and roll rates (with an anti-roll bar), ride frequency, roll gradient,
  lateral load-transfer distribution, and dive and squat in braking and acceleration. It also checks the
  rod's clearance to the other arm, the tie rod and the rim.
* **Spring layout generator.** From a damper, a motion-ratio and ride-frequency target and a box for the
  rocker, it generates the rod, rocker and damper points. It then optimises them for motion ratio,
  progression, travel to the damper stops, lever angles, rocker rotation and rod clearance. Finally it
  picks spring rates you can buy and sizes the anti-roll bar for a load-transfer or roll-gradient target.
* **Tolerance sensitivity.** Moves every hardpoint coordinate by its build tolerance and shows which ones
  move roll centre, camber gain, bump steer, anti, caster, KPI and Ackermann most, against allowances.
* **Setup and rules.** Camber per mm of shim, toe per turn of the tie rod, ride height per mm of pushrod or
  spring perch, and checks of wheelbase and wheel travel against the rules.
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

# 4. 3D check (edit the `three_d:` blocks first: caster, anti %, pivot spacing, tie rod).
#    It also prints the springs (if the design has `spring:` blocks), the adjustments and the rules check.
fsae-susp check3d designs/mycar_opt.yaml -o results3d/mycar_opt

# 5. Design the springs: add a `spring_design:` block (damper, targets, rocker box; see the example), then
#    generate and optimise the rod, rocker and damper, and pick the spring and bar rates (about 1 min per
#    axle; --quick about 10 s). The result is a new design file with `spring:` blocks.
fsae-susp design-springs designs/mycar_opt.yaml -o designs/mycar_springs.yaml --plot

#    Analyse springs (yours, or the ones just designed): motion ratio, rates, roll, pitch
fsae-susp springs designs/mycar_springs.yaml -o results3d/mycar_springs

# 6. Which hardpoints need tight tolerances: ±1 mm on the chassis, ±0.5 mm on the upright
fsae-susp sensitivity designs/mycar_opt.yaml --tol 1 --tol-upright 0.5 -o sensitivity/mycar_opt

# 7. Everything in one go, plus a Markdown report
fsae-susp report designs/mycar_opt.yaml -o report/mycar_opt

# Get the VSusp link back to view the design in the browser
fsae-susp link designs/mycar_opt.yaml
```

`examples/fsae_aero_v2.yaml` is a complete design (the 3D-checked aero redesign) with every field commented.

## GUI

`fsae-susp-gui` (or `python -m fsaesusp.gui`) opens a small window. Pick a design file or paste a VSusp
link, then click Analyze, Optimise, 3D check, Springs, Sensitivity, Full report or VSusp link. The GUI runs the
same commands as the command line. Their output appears in the Output tab, and the PNGs they write appear in the Plots tab.
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

Optimise... opens a dialog with each axle's pivot coordinates, ball-joint positions and static camber, and
the front outer tie rod and rack line, with their current values and search ranges. Tick the ones to lock.
The design file's `lock:` lists start ticked. A coordinate that the bounds already fix is greyed out. With a
steering target, tick Steering only to place just the tie rod and rack. Then pick where to save the new
design.

Sensitivity... asks for the chassis and upright tolerances (starting from the design file's `sensitivity:`
block) and the axle. It takes about 20 s per axle.

Springs... offers to analyse the design's `spring:` blocks, or to design new ones from its `spring_design:`
block. Designing asks where to save the new design (`<name>_springs.yaml` by default), then opens it, with
its springs plot.

Imported links are saved as `designs/<name>.yaml`, as on the command line. The name defaults to the VSusp
project name, and you can change it before saving.

The GUI chooses the output folders, using the design file's name for the subfolder. For `sastha.yaml`, Analyze
writes to `results/sastha/`, 3D check and Springs to `results3d/sastha/`, Sensitivity to `sensitivity/sastha/`
and Full report to `report/sastha/`. These folders are in the project root. Optimise saves the new design next to the original (`sastha_opt.yaml` by default) and
its plots in `sastha_opt_plots/`.

## Commands

| Command | What it does |
|---|---|
| `import-vsusp LINK [-o design.yaml]` | New design file from a VSusp link (or a text file holding it). Without `-o` it is saved as `designs/<VSusp project name>.yaml`. 3D and vehicle values start at defaults. |
| `link design.yaml` | Print the VSusp link for the design. |
| `analyze design.yaml [-o DIR] [--compare other.yaml]` | Front-view table and flags. With `-o`: `front_view.png` and `report_front_view.md`. `--compare` overlays a second design. |
| `compare a.yaml b.yaml [c.yaml d.yaml] [-o DIR]` | Front-view metrics of 2 to 4 designs side by side, with the change from the first to the second when there are two. With `-o`: an overlay `front_view.png`, `comparison.csv` and `comparison.md`. |
| `optimize design.yaml -o new.yaml [--axle front/rear/both] [--quick] [--workers N] [--plot] [--lock ...] [--unlock ...] [--steering-only]` | Optimise inner pivots (and ball joints given bounds, with an optional KPI target), then the front outer tie rod and rack line for any Ackermann or steering-ratio target, and write a new design file. `--workers -1` uses all cores. `--lock`/`--unlock` hold or free parameters (see below). `--steering-only` runs just the steering stage. |
| `check3d design.yaml [-o DIR] [--no-steering] [--sync]` | 3D tables and flags, then springs (if any), adjustments and the rules check. With `-o`: `hardpoints_3d.csv`, `check3d.png`, `layout_3d.png`, `springs.png` (with springs), `report_3d.md`. `--sync` writes the 3D tie-rod points back into the design file. |
| `springs design.yaml [-o DIR]` | Rod, rocker and damper of each axle with a `spring:` block, and ride, roll, load transfer and pitch when both have one. With `-o`: `springs.png` and `springs.md`. |
| `design-springs design.yaml -o new.yaml [--axle front/rear/both] [--quick] [--no-optimise] [--workers N] [--plot] [--lock ...] [--unlock ...]` | Generate and optimise the rod, rocker and damper from `spring_design`, pick the spring rates and size the anti-roll bar, then write a new design file. `--no-optimise` writes the generated layout without searching. `--plot` writes `springs.png` in `new_plots/`. |
| `sensitivity design.yaml [--tol MM] [--tol-upright MM] [--axle front/rear/both] [-o DIR]` | Hardpoint tolerance sensitivity in 3D (about 20 s per axle). With `-o`: `sensitivity.csv` (change per mm for every coordinate), `sensitivity.png` and `sensitivity.md`. |
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
  * `inner_x_offset` (mm, with `auto` or `{y: ...}`): puts the rack line this far ahead (+) of the outer tie rod
    instead of level with it. Moving the rack forward changes Ackermann and gives the tie rod more rim clearance
    at full lock.

  To design the arm instead, use `design: {side: rear|front, arm_length, arm_angle, below_hub}`.
  Then run `check3d --sync` to copy the resulting points back into the file.
* `steering.lock_inside`: the inside-wheel lock to check. A 9 m FSAE hairpin needs about 27°.
* `steering.c_factor` (optional): rack travel per steering-wheel turn, mm. With it, the 3D check reports the
  steering ratio (steering-wheel degrees per wheel degree at centre) and the optimiser accepts a
  `steering_ratio` target.
* `vehicle`: wheelbase, CG height, front brake bias and `inboard_drive`. These feed the anti percentages.
* `wheel.lip_inboard`, `lip_radius`, `link_radius`, `node_inboard_of_lip`: the clearance model (see below).
  **Measure your own wheel.**

`optimise:` holds the optimiser settings:

* `camber`: the outside-wheel target and the low- and high-speed (roll, heave) cases used to pick static
  camber.
* `front/rear.targets`: for example `rc_height`, `fvsa_range`, `lca_incl_range`, `rc_heave_slope_max`,
  `rc_lateral_tol`, `lower_arm_min`, `upper_arm_min`. The full list is `DEFAULT_TARGETS` in
  `src/fsaesusp/optimize.py`. Without `rc_height`, the current roll-centre height is the target. Also:
  * `kpi` (deg): static KPI to hold exactly, for example `kpi: 0`. It needs a ball joint that can move (see
    below). The upper joint is placed to hold it, or the lower one if the upper one is fixed. The static camber
    is then searched with the pivots, because KPI = upright angle − camber.
  * `scrub_target` (mm, default 0) and `scrub_tol` (default 15): scrub radius is part of the cost whenever
    a ball joint can move.
  * Front steering targets. Any of the first three starts the steering stage (see How it works):
    * `ackermann` (%, at `steering.ackermann_at`).
    * `rack_per_deg`: rack travel per degree of wheel steer at centre, mm/deg.
    * `steering_ratio`: steering-wheel degrees per wheel degree. It needs `steering.c_factor` and replaces
      `rack_per_deg`.
    * Limits: `tie_rod_clearance_min` (mm, tie rod to rim at full lock; default: the design's current value),
      `arm_tie_rod_angle_min` (deg, default 30) and `bump_steer_max` (deg over ±30 mm, default 0.05).
    * `ackermann_tol` (default 0.5 %) and `rack_per_deg_tol` (default 0.005 mm/deg) weight the misses.
    The defaults are `DEFAULT_STEERING_TARGETS` in `src/fsaesusp/steeropt.py`.
* `front/rear.bounds`: `lower_x`, `lower_y`, `upper_x`, `upper_y` as `[lo, hi]` in VSusp coordinates.
  `lo == hi` fixes that coordinate. Without bounds, the search stays within ±30 mm of the current pivots.
  * `upper_bj_x`, `lower_bj_x`: how far inboard of the hub face each ball joint may sit (the first number of
    `knuckle.upper_ball_joint` / `lower_ball_joint`). Without bounds the upright is left alone. Ball-joint
    heights never move.
  * Front steering stage, in mm:
    * `tie_rod_in`, `tie_rod_down`: outer tie rod inboard of the hub face and below the hub (`knuckle.tie_rod`).
    * `tie_rod_x`: outer tie rod ahead of the axle (`three_d.tie_rod.x`, negative behind).
    * `rack_x`: the rack line ahead of the axle (`tie_rod.x` + `inner_x_offset`).

    Without bounds, the outer tie rod may move ±15 mm across and fore/aft, and its height and the rack line
    stay put. The rack height and rack-end position always come from `inner: auto` (zero bump steer).
* `front/rear.lock`: parameters that keep the design's current value, for example `lock: [lower_y, camber]`.
  You can lock `lower_x`, `lower_y`, `upper_x`, `upper_y` (the pivot coordinates), `upper_bj_x`, `lower_bj_x`
  (the ball joints), `camber` (the static camber, which is otherwise chosen for `outside_target`) and, for the
  front, `tie_rod_in`, `tie_rod_down`, `tie_rod_x` and `rack_x`. A lock overrides that coordinate's bounds.
  Arm lengths can't be locked: they follow from the pivots, track and camber.

  For example, KPI 0° with the lower pivot x fixed and the ball joints free between 40 and 60 mm:

  ```yaml
  optimise:
    front:
      targets: {kpi: 0.0}
      bounds: {upper_bj_x: [40, 60], lower_bj_x: [40, 60]}
      lock: [lower_x]
  ```

  And Ackermann back to 75 % at 1.10 mm/deg, with the tie rod at least 8.6 mm off the rim and the rack free
  to move up to 25 mm forward:

  ```yaml
  optimise:
    front:
      targets: {ackermann: 75, rack_per_deg: 1.10, tie_rod_clearance_min: 8.6}
      bounds: {rack_x: [-60, -35]}
  ```

  `fsae-susp optimize designs/mycar.yaml -o designs/mycar_steer.yaml --steering-only` places just the steering.

  On the command line, `--lock` adds locks and `--unlock` removes ones the file sets. Give names as
  `[axle.]name`, comma-separated or repeated. A name without an axle applies to every axle being optimised:

  ```bash
  fsae-susp optimize designs/mycar.yaml -o designs/mycar_opt.yaml --lock lower_y,camber --lock rear.upper_x
  ```

  The new design file keeps the locks that were used, so optimising it again holds the same parameters.
  When camber is locked, arm lengths are rounded to 0.01 mm instead of 0.1 mm, so camber stays within about
  0.005°.

**Springs** (`front.spring`, `rear.spring`, optional). Points are 3D for the left corner, with **x measured
from that axle** (so rear points don't depend on the wheelbase), y to the left and z up:

* `rod`: where the pushrod or pullrod picks up. Either `{arm: lower|upper, at: 0.9, above: 10}`, meaning
  90 % of the way from the arm's pivot axis to its ball joint and 10 mm above it, or
  `{arm: lower|upper|upright, point: [x, y, z]}`. Take the arm points from `hardpoints_3d.csv`.
* `rocker`: `pivot`, `axis` (the direction it turns about), and the `rod` and `damper` points on it.
* `damper`: the `chassis` end, and optionally `length: [compressed, extended]` eye to eye, which gives the
  wheel travel to the damper's stops.
* `spring_rate` (N/mm), coaxial with the damper.
* `arb` (optional): `{wheel_rate: N/mm}` at each wheel in roll, or `{bar_rate: Nm/deg, lever: mm,
  motion_ratio: ..}`. The bar's motion ratio can instead come from a drop link: add `arb: [x, y, z]` to the
  rocker and `link: [x, y, z]` (the bar arm's end) here.

The ride, roll and pitch numbers need both axles' spring blocks, plus `tyre.rate` (vertical rate, N/mm) and
`vehicle.mass` (kg, with driver), `front_weight`, `unsprung_mass` (kg per corner, one value or
`[front, rear]`), `lateral_g`, `braking_g` and `accel_g`. Missing vehicle values fall back to defaults and are
flagged. If the springs give a different roll at `lateral_g` than `optimise.camber.low_speed` assumes, that is
flagged too, so the optimiser can pick static camber for the roll the car really has.

**Spring design** (`spring_design:`, optional) drives `design-springs`. Per axle:

* `damper: [compressed, extended]`: eye-to-eye lengths from the damper's data sheet.
* `arm: lower|upper` (pushrod or pullrod), `above` (pickup height over the arm, mm), and `damper_dir`: roughly
  which way the damper runs from the rocker to the chassis, for example `[0, -1, 0]` across the car.
* `targets`:
  * `mr`: static motion ratio. By default it uses 95 % of the stroke over the travel.
  * `ride_frequency` (Hz): picks the spring rate. Without it, the `spring_rate` already in the axle's spring
    block is kept.
  * `travel: [bump, droop]` and `travel_margin`: what the damper stroke must allow.
  * `progression: [lo, hi]` (%): motion-ratio change from −25 to +25 mm.
  * `min_angle`, `max_rotation`, `rod_clearance`, `rim_clearance`, `rod_force_max`.

  The defaults are `DEFAULT_TARGETS` in `src/fsaesusp/springdesign.py`.
* `bounds`: `[lo, hi]` for each layout variable, with `lo == hi` fixing it.
  * `at`: the rod pickup along the arm.
  * `pivot_x`, `pivot_y`, `pivot_z`: the box the rocker pivot may sit in. x is from this axle. These are
    needed unless the axle already has a spring block, whose pivot ±25 mm is then used.
  * `rod_lever`, `damper_lever`: the lever lengths.
  * `rod_lever_angle`, `damper_lever_angle`: each lever turned from square to its line.
  * `damper_tilt`, `damper_swing`: the damper's direction.
* `damper_box: {x: [..], y: [..], z: [..]}`: where the damper's chassis end may go.
* `lock`: keep the spring block's current `at`, `pivot_x`, `pivot_y`, `pivot_z`, `rod_lever` or
  `damper_lever`. On the command line, `--lock` and `--unlock` work as for `optimize`.

Shared settings:

* `spring_step` (lb/in, default 25) rounds the spring rates. `spring_rates: [N/mm, ...]` picks from a list
  instead.
* `arb: {axle: front, lltd_front: 0.50}` sizes the anti-roll bar for a front lateral load-transfer
  distribution. Use `roll_gradient: deg/g` instead to size it for a roll gradient.

The program only knows the links, the wheel and the boxes you give it. **Check that the rocker, the damper and
their mounts clear the chassis in CAD.**

**Other optional blocks:**

* `sensitivity: {tol: 1.0, tol_upright: 0.5, allow: {toe_bump: 0.05, rc_height: 5, ...}}`: default tolerances
  (mm) and the allowance for each metric. The defaults are `DEFAULT_ALLOW` in `src/fsaesusp/sensitivity.py`.
* `adjust: {thread_pitch: 1.058, turnbuckle: false}`: the tie-rod thread (default 3/8-24 UNF). With
  `turnbuckle: true` a turn changes the length by two pitches.
* `rules: {wheelbase_min: 1525, travel_min: 25, track_ratio_min: null}`: the rules check. `null` switches a
  check off. These are defaults: check them against the current rules of your competition.

## How it works

* **Front view** (`geometry2d.py`): a rigid knuckle on two arms, with the tread centre held on the ground.
  This is VSusp's model, solved by root-finding on the lower-arm angle. Roll rotates the body about the
  frame-bottom centre with both tyres on the ground, as in VSusp.
* **Optimiser** (`optimize.py`): differential evolution, then a bounded Nelder-Mead polish, over the four
  pivot coordinates. Arm lengths are derived so track and static camber hold. The cost is a sum of squared
  normalised misses: RC height, RC heave slope above a threshold, RC lateral migration, swing-arm band, RC
  at the heave case, half-track change and packaging (arm lengths, arm angles, frame depth). Afterwards,
  static camber is chosen so the outside wheel averages `outside_target` over the two cases, unless it is
  locked. Free pivots are rounded to 0.5 mm and arm lengths to 0.1 mm.
  * **Ball joints and KPI.** Ball joints with a range join the search, and scrub radius joins the cost.
    * With a `kpi` target, one joint is not searched. It is placed in closed form from the other joint and
      the camber: hux − hlx = (huy + hly) · tan(KPI + camber). The static camber becomes a search variable,
      with the outside-wheel miss in the cost.
    * If that joint ends up past its limit, it is held at the limit and the other joint holds the KPI.
    * The searched joint is rounded to 0.5 mm, the placed one to 0.01 mm and arm lengths to 0.01 mm, so
      KPI holds to about 0.005°.
  * VSusp sets the lower ball joint at whole-mm heights. It can therefore show KPI and camber a few
    hundredths of a degree off this continuous solution (for example −0.04° for a design at 0.00°).
* **Steering stage** (`steeropt.py`): after the front pivots, with an Ackermann or steering-ratio target.
  * Bounded least squares over the free steering variables. Each misses Ackermann and rack travel per degree
    by its tolerance, and the limits act only when crossed.
  * A small pull back toward the current layout picks, among the layouts that meet everything, the one that
    moves least. The rack moves forward only as far as the clearance limit needs.
  * Each layout is built with `inner: auto` (or `{y: ...}`), so bump steer stays at its minimum.
  * Clearance is checked at 70 % and 100 % of lock each way at −15, 0 and +15 mm heave.
  * Values are rounded to 0.1 mm. A full layout takes about 2 s, so the stage takes 1–2 min.
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
* **Springs** (`springs.py`): the rod pickup rides on the arm (or upright) as a rigid body. At each wheel
  position, the rocker angle that keeps the rod length is solved in closed form, which gives the damper
  length. Motion ratio = −d(damper length)/d(wheel travel). Wheel rate = spring rate × MR². Ride and roll
  rates put the tyre in series. Roll stiffness = wheel rate in roll × track² / 2. The roll gradient uses the
  sprung mass and the roll axis under the sprung CG (from the front-view roll centres). Lateral load
  transfer splits into elastic (by roll stiffness), geometric (roll-centre height) and unsprung parts. In
  braking and acceleration, the longitudinal transfer less the anti share goes through the ride rates.
* **Spring design** (`springdesign.py`):
  * The wheel's path is solved once, every 1 mm over ±60 mm. Each candidate layout then needs only the
    closed-form rocker angle at each position, so an evaluation takes about 3 ms.
  * Construction:
    * The rocker plane holds the rod pickup, the pivot and the damper line.
    * The rod meets its lever where it is tangent to the lever circle, so it is square at static.
    * The damper lever is square to the damper line, on the side that compresses the damper in bump.
    * The static damper length is placed in the stroke so the wheel has the same margin past the bump and
      droop targets.
  * The generated layout sizes the damper lever for the target motion ratio. Motion ratio = rod motion ratio
    × damper lever ÷ rod lever when both levers are square.
  * The search is differential evolution started from that layout, then Nelder–Mead. Its cost is the squared
    misses of the targets, plus small preferences for a stiffer rod path and less rocker rotation.
  * Rates:
    * Spring rate = m (2πf)², with the tyre taken out of series, ÷ MR², rounded to `spring_step`.
    * The anti-roll-bar wheel rate is solved for the load-transfer or roll-gradient target.
* **Sensitivity** (`sensitivity.py`): each of the 24 hardpoint coordinates (arm pivots, inner and outer tie
  rod, ball joints) is moved ±tolerance in turn.
  * *As built* keeps the nominal link lengths, so it shows how far the static camber and toe move, which is
    the adjustment range needed.
  * Everything else has the links made to fit, so static camber and toe are back at nominal and only the
    kinematics change.
  * The roll-centre height comes from the 3D contact-patch path (half-track × dy/dz), which matches the
    front view exactly with no caster or tilt.
  * Each metric reports a worst-case stack (sum) and an RSS stack. It is flagged when the RSS is over the
    allowance.
* **Adjustments** (`adjust.py`) change one thing at a time with the rest of the linkage as built:
  * a 1 mm shim at the upper ball joint;
  * 1 mm of tie-rod length;
  * 1 mm of ride height;
  * 1 mm of pushrod length, holding the damper length, since the spring load is unchanged;
  * 1 mm of spring perch, which moves the ride height by 1/MR.

## Validation

`pytest` runs the checks below (about 3 min):

* The front-view solver reproduces VSusp's own geometry to 0.0002 mm at the same lower-ball-joint height,
  using values read from vsusp.com for `examples/original_vsusp_link.txt`.
* VSusp links round-trip, and so do design files.
* With zero caster and no pivot-axis tilt, the 3D solver reproduces the 2D camber and contact-patch path.
* The example design's 3D results stay as checked: 20 % anti-dive, 25 % anti-squat, 51 % Ackermann, and
  zero bump steer.
* The optimiser respects its bounds and locks. Without ball-joint bounds or a KPI target, it leaves the upright
  alone and gives the same result as before.
* A KPI target is held to 0.01°, with the camber still on its outside-wheel target. A joint at its limit hands
  the KPI to the other joint. A KPI target with no free ball joint is refused before any search starts.
* The steering stage hits an Ackermann target within 1 % at the starting rack travel per degree. It keeps the
  tie-rod clearance and moves nothing it shouldn't: pivots, upright, the outer tie-rod height and the rack line
  all stay put. A steering ratio converts through the c-factor. A steering ratio without a c-factor, a design
  using `tie_rod.design`, or no free steering variable is refused before any search starts.
* The 3D roll-centre height, its heave slope and camber gain match the front view to 0.001 mm.
* Sensitivity: caster's change with the upper ball joint's x matches the closed form. The links made to fit
  leave the static alignment alone.
* Springs:
  * With the rod and damper square to their rocker levers, the motion ratio equals the lever ratio × the
    rod's own motion ratio, to 1e-6.
  * The damper reaches its stops at the reported travels.
  * Lateral load transfer × track, summed over both axles, equals m·g·h exactly.
  * Braking dive matches the hand calculation.
* Spring design:
  * The generated layout hits its motion-ratio target and uses the damper stroke exactly, with equal bump
    and droop margins.
  * The search's fast evaluation agrees with the full springs analysis of the block it writes: motion ratio
    to 0.003, travel to 0.3 mm, angles to 0.5°.
  * A quick search meets every target within its bounds, and locks hold the spring block's values.
  * Spring rates match the hand formula and round to the nearest 25 lb/in.
  * The sized bar hits a 50 % front load-transfer distribution.
  * The segment-distance helper matches brute-force sampling.
* Adjustments: a shim tilts the wheel by about 1 mm over the kingpin length, and the pushrod and perch
  ratios match 1 / motion ratio. The rules check passes and fails where it should.

## Limitations

* Double wishbone only. Other VSusp suspension types are rejected on import.
* Rigid links. There is no compliance, tyre lateral deflection or pneumatic trail.
* Symmetric car: the right side is the mirror of the left.
* Roll is a pure rotation about the frame-bottom centre with both tyres on the ground, and heave is
  body-fixed with the ground raised. This matches VSusp.
* Springs are linear and coaxial with the damper. The rocker turns on a fixed axis, there are no bump stops
  or damper forces, and the anti-roll bar is a rate (or bar rate and lever).
* Spring kinematics are for wheel travel with the body fixed. Roll stiffness uses the static motion ratio.
* The spring designer has no model of the chassis. The rocker and damper only stay inside the boxes you
  give. Rod clearance treats links as tubes of `wheel.link_radius`, with the A-arm legs meeting at the
  clearance model's node.
* Sensitivity is linear, one coordinate at a time, for one corner. The stacks assume independent errors.
* Clearance numbers depend on the simple wheel model above.

## Python API

```python
from fsaesusp import config, api
cfg = config.load("examples/fsae_aero_v2.yaml")
res = api.analyze(cfg)            # front view: res["front"]["metrics"]["rc_heave_slope"], ...
res3d = api.check3d(cfg)          # 3D: res3d["steering"]["ackermann_percent"], res3d["springs"], res3d["rules"]
sp = api.springs(cfg)             # sp["axles"]["front"]["mr"], sp["vehicle"]["roll_gradient"], ...
new, res = api.design_springs(cfg, quick=True)          # new["front"]["spring"], res["front"]["result"]["mr"], ...
sens = api.sensitivity(cfg, tol=1.0, tol_upright=0.5)   # sens["front"]["stack"]["toe_bump"]["rss"], ...
print(config.to_vsusp_link(cfg))
```
