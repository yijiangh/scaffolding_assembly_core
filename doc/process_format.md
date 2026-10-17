# Process file format, schema 2.1

Status: **implemented** (schema 2.1 of 2026-10-13; schema 2 with the decisions of 2026-10-09, then the changes listed below). Read and written by
`scaffolding_assembly_core.process`, part of the shared core `scaffolding_assembly_core` (plan: husky-assembly-teleop
`tasks/2026-10-08_shared_core.md` §12; decisions: `tasks/2026-10-09_core_integration/README.md`). This is the in-repo copy of the doc "Husky
design format: schema 2 proposal", which also gives the reasons and the examples from the 260814 export. Where the
proposal leaves a point open, the choice made here is marked **Resolved**.

**Schema 2.1 against schema 2** (a 2.1 reader refuses schema 2 files; export again):

| Change | Why |
|---|---|
| The container is a **process** (`process.json`, `Process`, format `husky_process`; was design, `design.json`) | The terms of Gramazio Kohler's integral timber joints and Huang et al. 2021: the assembly is the structure, the process is everything needed to build it robotically. |
| A body's pose in the finished structure is `assembly_pose` (was `pose`, the "design pose") | Says which pose it is; `State.poses` and a holder's grasp are other poses. |
| `present` lists every unmounted body, obstacles too (they were implied) | One rule: a body is present when listed, a mounted half when its bar is. An obstacle can now appear or go. |
| The jointing movement where the operator puts the bar into the tools is `place_bar` (was `mount`) | `mount` means one thing: a half fixed to its bar. |
| A tool id always names its robot: `tools/<robot>/<tool>` | Which robot carries a tool shows wherever the id appears; one rule instead of two. |
| `pose_in_tcp` on each connector half a tool acts on replaces the `parts` catalogue and its `seat` | A body reads alone, as it does for its shape; `part` is a label again. |
| `mount_contacts` is gone from tools | A planning detail: the arm links a mounted tool may touch follow from its flange. |
| Mesh files are `meshes/<owner>.<hash>.obj` | The name says what the mesh is of; the hash still covers the content. |
| A target is shaped as a state: `target.robots[<robot>]` holds `joints`, `link_poses` and `base` (was `target.joints`, `target.links`) | Start and target read the same way; the end of a movement is its start with the target laid over it. |
| A movement may drive a base: `base` lists the robots, the target gives where | The file can say it; no path type or planner handles it yet. |

Sections 1–10 define the format. Section 11 lists the checks. Appendices map it to compas_fab and to the old export,
and give an example.

---

## 1. Scope and principles

A **process** is the plan for one assembly: the robots, their tools and every body; the schedule; and per movement its
start state, its target and the parts that say what moves and how it ends.

1. **The file is the in-memory `Process`, field for field:** one dataclass per object of this format, named as its
   keys. Scenes, mirrors and host details are derived from it or live beside it, never in it. Only file plumbing
   differs: robot paths and their `files_hash`, mesh files, the `writer` stamp.
2. **The process is the plan:** planned poses, what moves, and how each movement ends. No planner status, no
   measurements. The one planner result it holds is a target's solved keyframe (§5.3 says why).
3. **Three products, each with one writer:** the process folder (Rhino), `solutions/` (planners), `runs/` (the
   monitor).
4. **Every state reads alone, together with the process:** bases, joints, attachments with their grasps, built bars and
   tool states. A pose is written only where it differs from what the process implies.
5. **One rule says where a bar is** (§8.2). Mounted halves follow their bar.
6. **`null` means "not decided by the process"**, for bases, joints and tool states alike. No placeholder values.
7. **Ids are readable and valid within one export.** Anything outside the process that refers to it records the
   content hash of what it refers to (§3.2).
8. **Anything a machine reads is a typed field.** `notes` is for people.

## 2. Folder layout

```
<process>/
├── process.json                  manifest: robots, tools, bodies, mates, schedule
├── actions/<action id>.json     one file per scheduled action
├── meshes/<owner>.<hash>.obj    written by the library; read: any path, .obj|.stl|.glb
├── robots/<robot>/robot.urdf, robot.srdf, meshes/…
├── source/                      optional: the .3dm, joint_pairs.json, robotic_tools.json it came from (never read)
├── solutions/<action id>.json   planner results (§10); never written by the exporter
└── runs/<yyyy-mm-dd_hhmmss>/    reserved for the monitor (§10)
```

The library's `write` leaves `source/`, `solutions/` and `runs/` as they are.

## 3. Conventions

| Item | Rule |
|---|---|
| Encoding | UTF-8 JSON. No comments, no `NaN`/`Infinity`. |
| Units | Metres and radians. |
| Pose | `[x, y, z, qx, qy, qz, qw]`: position, then unit quaternion (x, y, z, w). World frame unless stated otherwise. |
| Robot base pose | Pose of the URDF root link in world. |
| `null` | Not decided by the process (bases, joints, tool states), or unknown after a manual step. |
| File references | Paths relative to the process folder, with `/`. |

### 3.1 Ids

Ids are the names Rhino writes: `[A-Za-z0-9_.-]+` segments joined by `/`, case sensitive, unique within one export.
A renumber or rename is a process change: re-export, then re-plan. Bodies get no separate label; `label` is optional
display text and contains no `/`.

| Prefix | Object | Example |
|---|---|---|
| `robots/<robot>` | robot | `robots/cindy` |
| `robots/<robot>/<link>` | one URDF link of a robot | `robots/cindy/left_ur_arm_tool0` |
| `tools/<robot>/<tool>` | tool mounted on that robot | `tools/cindy/AT3L`, `tools/alice/SupportGripper` |
| `bars/<bar>` | bar | `bars/B7` |
| `joints/<joint>` | connector half | `joints/J3-10_male`, `joints/G1-T20Ground-0_ground` |
| `ground/<ground>` | walkable ground surface | `ground/WG0` |
| `obstacles/<name>` | other static body | `obstacles/column_A` (Rhino: the object's name; `env<n>` with an export warning when it has none) |

Action and movement ids are plain names without `/`, unique per process. Producers name them by role, so people and
programs find a movement without counting: an action is `<bar>_<code>` (`J` jointing, `R` release, `H` holding, `HR`
holding release, `P` placing by hand), a movement `<action>_<role>`, with `_2`, `_3`, … when a role repeats.

| Action | Roles, in order |
|---|---|
| `B10_J` | `load`, `place_bar`, `grasp`, `transfer`, `insert` |
| `B10_R` | `ungrasp`, `retreat`, `home` |
| `B3_H` | `approach`, `open`, `to_grasp`, `close` |
| `B3_HR` | `open`, `retreat`, `leave` |
| `B2_P` | `place` |

The role is a naming convention: what a movement does is in its typed fields.

### 3.2 Content hashes

The content hash of a file is the SHA-256 of its JSON with `writer` and `producer` removed, keys sorted, floats rounded
to 12 decimals (`process.content_hash`). The writer is deterministic (ids sorted, the same rounding), so re-exporting an
unchanged process gives the same hashes, also from a newer library commit. `process.json` covers every file it refers
to: meshes are named by their content, and each robot records the hash of its files (`files_hash`).
`process.process_hashes(process)` gives the hashes a `write` would give without writing, so a host can tell a stale
solution while the process is still in memory. Readers never match by id alone:

| File | Records | On a mismatch |
|---|---|---|
| `solutions/<action>.json` | `solved_against`: hashes of `process.json` and of the action file | Stale: shown, never executed |
| `runs/<run>/run.json` | Hashes of `process.json`, of each executed action file and of each solution | Read against the process version it executed |
| Planner caches | The same two hashes in the cache key | Rebuilt |

## 4. `process.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_process"` | yes | File kind. |
| `writer` | Writer (§9) | yes | The library that wrote the file. |
| `producer` | Producer (§9) | no | The code that made the process, e.g. Rhino's `RSExportProcess`. |
| `robots` | map robot id → Robot | yes | Every robot. |
| `tools` | map tool id → Tool | yes | Every tool; may be empty. |
| `bodies` | map id → Body | yes | Every body. |
| `mates` | array of [body id, body id] | no | Joints of the finished structure (§8.3): a male and a female half, or a ground half and a ground body. |
| `schedule` | array of action ids | yes | Execution order across all robots: the only order. |

### 4.1 Robot

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `urdf`, `srdf` | path | yes | Robot description; planning groups, passive joints, disabled collisions. |
| `files_hash` | string | no | SHA-256 of the URDF, the SRDF and every mesh the URDF names, as copied into the process. Written by the library; a mismatch on read is an A4 error. |
| `serial` | string or `null` | no | Hardware serial (`"0806"`). |
| `tools` | map link name → tool id | no | Tools mounted on this robot, by flange link. |
| `ground_links` | array of link names | no | The links the robot stands on (its wheels): they may touch any `ground/` body. |

### 4.2 Tool

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `collision` | array of Shape | yes | Collision shapes in the flange link frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `tcp` | Pose | yes | Tool centre point in the flange link frame. |
| `kind` | string | yes | Hardware type, from the tool vocabulary (§7). |

### 4.3 Body

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `assembly_pose` | Pose | yes | Where the body stands in the finished assembly, in world. Bars: where they are once built. |
| `collision` | array of Shape | yes | Collision shapes in the body frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `label` | string | no | Display text. Absent: the id. |
| `part` | string | no | What kind of half it is, e.g. `"T20/Female"`, `"T20/MoCap"`, `"T20/Ground"`: a label, for people and mocap. |
| `markers` | map label → [x, y, z] | no | Marker points in the body frame, metres: placement checks, mocap registration. |
| `mount` | bar id | halves only | The bar a connector half is fixed to: rigid and static for the whole process; the half follows its bar. Required on every `joints/` body. |
| `pose_in_tcp` | Pose | halves only | For a half a tool acts on: the half's pose in the frame of the tool centre point on it (identity when the two frames coincide; a `T20/Ground` half sits turned 180° about z). Absent: no tool goes on this half. B14 checks grasps against it. |

The offset of a half on its bar follows from the two assembly poses. Bars do not list their halves:
`Process.halves_of(bar)`. A bar with its mounted halves moves as one piece ("the whole part" in §8.4); it is not stored. A body under `ground/` is walkable.

### 4.4 Shape

Exactly one geometry key, plus an optional `origin`: `{"mesh": <path>}` (§6), `{"box": [sx, sy, sz]}` centred at
`origin`, `{"cylinder": [radius, length]}` along the Z axis of `origin`, centred at it. `"origin"`: the shape's pose in
the owner's frame; absent: identity.

## 5. Action file `actions/<action id>.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_process/action"` | yes | File kind. |
| `writer` | Writer (§9) | yes | |
| `id` | action id | yes | Equals the file name without `.json`. |
| `type` | enum | yes | `bar_jointing`, `bar_release`, `bar_holding`, `bar_holding_release`, `bar_placing`. |
| `robot` | robot id | yes | The acting robot. `""` for `bar_placing`: the bar is placed by hand, in one manual step (`ends_on: operator`) whose target adds it to `built`. |
| `bar` | bar id | yes | The bar the action is about. |
| `ground` | array of ground ids | no | Ground surfaces the base may stand on. |
| `supports_until` | array of bar ids | no | `bar_holding` only: bars that must be built before the hold is released. |
| `label` | string | no | Display text. |
| `notes` | object | no | For people (§5.4). |
| `movements` | array of Movement | yes | In execution order. |

### 5.1 Movement

A movement is one segment that runs without stopping, described by independent parts. Its kind follows from the parts
and is not stored: an **arm move** has arms only; a **tool move** a grip change or a drive only; a **combined move**
both (insert, the compliant forms of ungrasp and untighten); a **manual step** neither, and `ends_on: operator`.
A `bar_placing` action is one manual step whose target adds its bar to `built`: a bar no robot handles, put into
the structure by hand at its place in the schedule (absent before it, built after it).

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `id` | movement id | yes | |
| `label` | string | no | Display text. |
| `arms` | array of link ids | no | Flange links of the arms that move. Absent: no arm moves. |
| `base` | array of robot ids | no | Robots whose base drives during the movement (the acting robot). Absent: no base moves. `path` and `controller` describe the arms only: **no path type for a base exists yet**, and planners report such a movement as not planned. |
| `path` | `free`, `linear` | when arms move | Path shape. |
| `coupled` | bool | no | The arms keep their relative pose, because they hold one bar. Default `false`. |
| `controller` | `position`, `compliant` | when arms move | `compliant`: the arm may stop short of its target. |
| `line` | map flange link id → {`direction`, `distance`} | on linear paths | One per moving arm: unit direction (world frame) and distance (m). |
| `drives` | map tool id → `tighten`, `loosen` | no | Run the jointing screw during the movement. |
| `ends_on` | `target`, `tools`, `operator` | no | Default `target`. |
| `start` | State (§5.2) | yes | |
| `target` | Target (§5.3) or `null` | no | `null` or absent: end where the next movement starts. |
| `notes` | object | no | For people (§5.4). |

**Rules**

1. When a movement ends: `target`: every moving arm reached its target and every tool part finished; `tools`: every
   tool part finished, the arms stop wherever they are; `operator`: the operator confirmed.
2. `ends_on: tools` needs a tool part; with arm motion it also needs `controller: compliant`.
3. `null` bases, joints and tool states are allowed anywhere: the process does not decide them. A planner may choose
   them offline and writes what it chose into its solution (`bases`, `start`); the monitor fills them from
   measurements at execution. Planning mirrors park other robots whose base or joints are `null`;
   `scenes.seeded` fills in the robot being planned for.
4. `target: null` means "end where the next movement starts".
5. `attached` and `built` may change within a movement; start and target say which: a manual mount or a grasp
   attaches the bar, an insert ends with it built, an ungrasp ends with it no longer attached, an untighten ends with
   it no longer built.
6. The process says what happens; the controller says how: motor start order, streaming rate, gains, force, stall
   detection and timeout lengths, the STOP that clears a stalled motor.

**Cindy's four screw operations.** Each is one movement, or two when the arms move only after the tool part (form B).

| Operation | Bar at start | Arms | Tool part | Ends on | Bar at end |
|---|---|---|---|---|---|
| Grasp | attached (resting in the tools), not built | none | grip → closed | stall | attached, not built |
| Insert | attached, not built | `linear`, `coupled`, `compliant`, forward | drive `tighten` | stall | attached and **built**: Cindy is frozen |
| Ungrasp, after an insert | attached and built | Form A: `linear`, `compliant`, back. Form B: none, then a careful retreat | grip → open, and drive `loosen` to back the jointing screw off | timeout | built, **no longer attached** |
| Untighten, to remove a bar | attached and built | Form A: `linear`, `coupled`, `compliant`, back. Form B: none, then a retreat that moves the bar | drive `loosen` | timeout | attached, **no longer built** |

### 5.2 State

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `robots` | map robot id → RobotState or `null` | yes | Every robot. `null`: absent: the state does not place it, and a scene disables it (its ground links may still touch the ground, §8.4). |
| `present` | array of body ids | yes | Every body that exists now and is not mounted on another: bars, ground, obstacles. A mounted body (a half) is never listed: it is present exactly when its bar is. There is no override: a present bar has all its halves. |
| `attached` | map bar id → array of Holder | no | Bars held by a robot, each holder with its grasp (§8.1). |
| `built` | array of bar ids | no | Bars fixed in the structure: they take their assembly pose. |
| `poses` | map bar id → Pose | no | Only present bars that are neither built nor attached and away from their assembly pose (a pickup rack). |
| `tools` | map tool id → ToolState or `null` | yes | Every mounted tool. `null`: its robot is absent and the state is not given. |

- RobotState: `base` (Pose or `null`) and `joints` (map joint → value over every non-passive joint, or `null`).
- Holder: `to` (link id `robots/<robot>/<link>`) and `grasp` (the bar's pose in that link's frame). A bar held through
  a tool is held by the flange with the composed grasp. The first holder of an unbuilt bar sets its pose; the others
  must agree.
- ToolState: `grip` (`open`, `closed` or `null`) and `on` (the body the tool is acting on, or `null`), both always
  written.

### 5.3 Target

A target is a partial State: the same keys with the same shapes, holding only what the movement decides. The state at a movement's end is its start with the target laid over it (`plan_check.end_state`). `present` and `poses` are never written in a target: they follow from the rest.

| Key | Type | Meaning |
|---|---|---|
| `robots` | map robot id → RobotTarget | The robots the movement moves, each with what it sets (below). A robot left out does not move. |
| `tools` | map tool id → {`grip`, `on`} | Each key optional, at least one. `grip`: the grip the tool is commanded to (commanding the grip it has is allowed: the hardware repeats it). `on`: the body it is on at the end, or `null` when it leaves; only where `on` changes. |
| `attached` | map bar id → array of Holder | **Resolved:** the whole `attached` map at the end, written only when the movement changes it. |
| `built` | array of bar ids | **Resolved:** the whole `built` list at the end, written only when the movement changes it. |

- RobotTarget: the keys of a RobotState, each optional. `joints`: map joint → value at the end; a subset is allowed, the other joints keep their start value. `base`: where the base ends, only for a robot the movement lists in `base`. `link_poses`: map link id → world pose at the end, e.g. of a flange; a start state has none, because its joints give them.

**Why a target carries solved joints.** A target often gives both `link_poses` and `joints` for the same arm: the
pose is what the process asks for, the joints are one inverse-kinematics solution that reaches it. That puts a
planner's output inside the process (against principle 3) and stores one fact twice. It is deliberate:

- Which solution is used is a design decision. An arm reaches a pose in several ways (the base here or there, the
  elbow up or down); the choice is made in Rhino, by a person, with collisions checked. A planner must not redo it.
- Reach is part of the process. A bar no robot can reach is a design error, so the proof of reach travels with the
  process.
- Motion planners need fixed ends: they plan between these keyframes and may not move them (§10, "the process wins").

The price: the two drift apart when a bar, a joint or a tool moves after the solve. B15 detects exactly that and
reports stale IK.

### 5.4 Notes

`notes` on actions and movements are for people: a key maps to a string, number or boolean, nothing nested. The library
never reads them (a core test fails if a core module does). A note a program needs is a missing field: propose a schema
change. Where the 14 keys of the old export go:

| Key | Schema 2 |
|---|---|
| `lm_axis`, `lm_distance_mm`, `retreat_axes_world` | `Movement.line` |
| `ends_on` | `Movement.ends_on` |
| `constraint` (`fixed_relative_ee_transform`) | `coupled: true` |
| `bar_arm_side` | follows from `attached` |
| `planner_fills`, `start_config_is_none` | `joints: null` |
| `unplanned_offline`, `goal_backfilled_from` | planner status: `solutions/` |
| `approach_axis`, `approach_offset_mm`, `retreat_target_config`, `after` | stay as notes |

## 6. Mesh files

| Rule | |
|---|---|
| Formats | `.obj` (text), `.stl` (binary), `.glb`. Chosen by extension. |
| Content | One mesh per file, in metres, in the frame of the shape that references it. |
| Sharing | Any number of shapes may reference one file. Readers load it once. |
| Names | The library writes `meshes/<owner>.<hash>.obj`: `<owner>` is the half's `part` (`T20-Male`), else the last segment of the first id using the mesh (`AT3L`, `WG0`); `<hash>` is 8 hex digits of the content, so equal meshes share one file and `process.json`'s hash covers them. Readers take any name. |
| Robot meshes | URDF mesh references are paths relative to the URDF file. |

## 7. Tool vocabulary

Every tool has one lasting state, its grip; Cindy's tools also have one drive, the jointing screw
(`scaffolding_assembly_core.process.vocabulary`). The vocabulary is part of the schema: adding a kind, a grip value or a drive
raises the schema number. Motor activity (`IDLE`, `TIGHTENING`, `LOOSENING`, `STALLED`) is execution data for `runs/`.

| Kind | Channel | Values | Hardware | How a change ends (controller) |
|---|---|---|---|---|
| `scaffolding_v3` | `grip` | `open`, `closed` | Gripper screws, motor M1 | `closed`: stall; `open`: timeout |
| `scaffolding_v1` | `grip` | `open`, `closed` | UR tool digital output (`set_io`) | output switched |
| `robotiq` | `grip` | `open`, `closed` | 2F-85 (`GripperCommand`) | goal reached, or stalled on the bar |

| Kind | Drive | Directions | Hardware | Ends on (controller) |
|---|---|---|---|---|
| `scaffolding_v3` | jointing screw | `tighten`, `loosen` | Motor M2 | `tighten`: stall; `loosen`: timeout |

A grip change is a `grip` in `target.tools`; it names the tools the movement commands. The tightness belongs to the
mate, not the tool. A `loosen` drive leaves the bar built, unless the target clears `built` (an untighten).

## 8. Bar states and relations

### 8.1 Bar states

| `attached` | `built` | Meaning | The robot holding it |
|---|---|---|---|
| yes | no | Carried: the bar follows the robot | Moves freely, carrying it |
| yes | yes | Fixed and held (after an insert, while a support robot grips it, before an untighten) | Frozen, except in the compliant movement that releases it |
| no | yes | Part of the structure | — |
| no | no | Staged: its pose is in `State.poses`; without one it floats at its assembly pose (warning) | — |

- `attached` means a robot holds the bar: set when a grip closes on it or the operator mounts it, cleared when the grip
  opens. Several holders: one robot's flanges moving `coupled` for an unbuilt bar (Cindy's left and right); different
  robots for a built bar (Cindy and Alice). Only bars are attached; halves follow by mount.
- `built` means the bar is fixed in the structure: set when one of its halves is mated (the end of an insert), cleared
  by an untighten.

### 8.2 Where a body is (`relations.placement`)

- A built bar takes its assembly pose and is never written in `poses`.
- An attached, unbuilt bar follows its first holding link's pose times the grasp.
- Any other present bar takes its pose in `State.poses`, else its assembly pose.
- A mounted half follows its bar by the mount offset (its own assembly pose when the bar is at its assembly pose).
- Ground and obstacles take their assembly pose.

### 8.3 Relations and mate status

| Relation | Between | Moves bodies? | Stored as |
|---|---|---|---|
| Attached | bar → robot link(s) | Only if the bar is not built | `State.attached` |
| Mounted | connector half → its bar | Yes, rigidly, for the whole process | `mount` on the half |
| On | tool → body | No: collision information only | `on` in `State.tools` |
| Mated | half ↔ half, or ground half ↔ ground | No | `mates` in `process.json` |

`on` is set by the target of the movement that brings the tool to the body (the operator's mount, or a support's
linear approach) and cleared by the target of the movement it leaves in (a retreat or a compliant back-off); it never
changes between movements (B11). A Cindy tool is on the half at its TCP (the male or a ground half); a
support gripper is on the bar it supports.

A mate's status follows from the `built` and `attached` flags of its two sides (`relations.mate_status`); a ground
body counts as built.

| Status | When | Contact |
|---|---|---|
| Engaged | Both sides built (or a built bar and the ground) | allowed |
| Pending | One side built, the other present, attached and not built (being joined or removed) | allowed between these two halves |
| Open | One side built, the other not present; **Resolved:** also every other combination | none: an ordinary obstacle |
| Not relevant | Neither side present | not in the scene |

### 8.4 Allowed contacts

Derived, never stored (`relations.allowed_contacts`):

| Rule | Derived from |
|---|---|
| A tool may touch the whole part it is on: the bar and every half mounted on it | `on`, `mount` |
| A half may touch the bar it is mounted on | `mount` |
| The halves of a pending or engaged mate may touch each other, and each other's bar | `mates`, `built`, `attached` |
| A half mated to a ground body may touch every present `ground/` body | `mates`, the body's prefix |
| A robot's ground links may touch any present `ground/` body, also where the state leaves the robot out (it still stands on the floor) | `ground_links`, the body's prefix |
| A tool may touch the last links of the arm it is mounted on | the flange it is mounted on (not stored: `urdf.TOOL_TOUCHES_ARM_LINKS`) |

The part rules are as wide as the parts because collision models are coarser than the parts: compas_fab checks each
tool as one convex hull, which overlaps the bar it holds by about 1 mm (the real meshes are 1.0 mm apart).

## 9. Writer and producer

```json
"writer": {"schema": 2.1, "library": "scaffolding_assembly_core", "commit": "3f9c2e71b0ad", "dirty": false},
"producer": {"repo": "bar_joint_rhino_design_workflow", "commit": "9a1b…", "dirty": false, "command": "RSExportProcess"}
```

`writer` names the library that wrote the file: `schema`, `library`, `commit` (or `"unknown"`), `dirty`. A reader
accepts only its own `schema` and names `commit` otherwise; the library converts nothing between schemas. `producer`
names the code that made the process. Neither is part of the content hash.

## 10. Solutions and runs

Neither ever changes the process, and the process never refers to them.

### 10.1 `solutions/<action id>.json` (`process.solutions`)

| Key | Type | Meaning |
|---|---|---|
| `format` | `"husky_process/solution"` | File kind. |
| `writer` | Writer | As in the process files. |
| `planner` | object | `name` (required), `repo`, `commit`, `ik_backend` (e.g. `ssik`), `artifacts` (the ssik artifact set or URDF it was built for), `settings` (any JSON). |
| `action` | action id | The action solved; equals the file name. |
| `solved_against` | `{process, action}` | Content hashes of `process.json` and of the action file as read. A mismatch marks the solution stale. |
| `movements` | movement id → Result | One entry per movement of the action. |

| Result key | Type | Meaning |
|---|---|---|
| `status` | enum | `solved`, `keyframe_only` (base and end joints, no trajectory), `failed`, `not_planned`. |
| `reason` | string | Why it failed; required for `failed`. |
| `bases` | robot id → Pose | Bases the planner chose where the process had `null`. |
| `start` | robot id → joints | Solved start joints. |
| `start_overridden` | bool | The planner moved a start the process had fixed; neighbours must agree with it. |
| `end` | robot id → joints | Solved end joints. |
| `trajectory` | object or `null` | `robot`, `joint_names`, `positions`, optional `times`: one trajectory per robot over all its moving joints. |
| `path_poses` | link or body id → [Pose] | Optional: a flange's or the attached bar's path, for showing the bar. |

- **Chaining.** Each movement starts where the previous one ended, within 1e-3 (radians or metres: the planners' IK
  tolerance), across actions in `schedule` order; the reader reports every break (`solution_warnings`).
- **The process wins.** Where the process gives target joints or a base, a solution's end must reach them (1e-3; a
  planner snaps its last waypoint to them); `bases`, `start` and `keyframe_only` only fill what the process leaves
  `null`. A keyframe chain may span one bar's J and R actions, on one base.
- **No arm motion.** A movement no arm moves in (a grip, a drive, a manual step) is `solved` with `start` equal to
  `end` and no trajectory (`MovementResult.still`), written by whoever plans the action.
- **Latest only.** One file per action holds the latest result; history lives in dated copies or in git.
- **Partial chains are normal.** Unplanned movements say `not_planned`.

### 10.2 `runs/<yyyy-mm-dd_hhmmss>/`

Reserved for the monitor; specified later. Planners never read it.

## 11. Validation

Two groups of checks, run at different times. Every message starts with its check, e.g. `A9: …` or `B7: …`.

| Group | Runs in | When | On failure |
|---|---|---|---|
| A, file checks | `scaffolding_assembly_core.process` (`read`, `write`, `validate`; `read_solutions`, `write_solution` for A14) | On every read and write | The file is rejected; every problem is reported at once |
| B, plan checks | `scaffolding_assembly_core.process.plan_check.check_plan` | On demand: Rhino before export, planners before planning, the monitor before execution | Errors block execution; warnings are reported |

### 11.1 A: file checks

| # | Check |
|---|---|
| A1 | Every file is UTF-8 JSON without `NaN` or `Infinity`, and `format` and `writer.schema` match in every file |
| A2 | Every required key is present with the right type, and no unknown key appears outside `notes` |
| A3 | Ids match the id pattern, are unique and use a known prefix; labels contain no `/` |
| A4 | Every reference resolves: robot, tool, body, half and action ids, and every file under the process folder (meshes, URDF, SRDF, and the URDF's mesh paths, relative to the URDF). A robot's `files_hash` matches its files |
| A5 | `schedule` and `actions/` match one to one, and each action file's `id` equals its file name |
| A6 | Each tool is mounted on exactly one robot, at a link its URDF declares, and its id `tools/<robot>/<tool>` names that robot |
| A7 | Every pose has 7 numbers and a unit quaternion (tolerance 1e-6) |
| A8 | Each state lists every robot, as a `RobotState` or `null`. A `joints` map names only joints the robot's URDF declares, and lists all non-passive ones |
| A9 | In each state, `present` lists only bodies that are not mounted (bars, ground, obstacles); `attached`, `built` and `poses` name only present bars; `poses` never names a built or attached bar; every holder has a link id `to` and a `grasp` |
| A10 | Every state lists every mounted tool. `grip` values and `drives` directions come from the vocabulary of that tool's kind, and `on` (in states and targets) names an existing body or is `null` |
| A11 | A target gives a `base` only for a robot the movement lists in `base`, and `base` lists only the acting robot. `path` and `controller` are present exactly when `arms` is not empty; `ends_on` is one of its values; `line` appears only on linear paths, with one entry per moving arm (unit direction, positive distance) |
| A12 | Every half's `mount` names an existing bar (only halves have one). `mates` pairs two halves, or a half and a ground body, and each half is in at most one mate |
| A13 | `notes` values are strings, numbers or booleans |
| A14 | Every `solutions/` file names an existing action (and its movements, with known statuses) and carries `solved_against`. A hash mismatch makes the solution stale, which is reported as a warning |

### 11.2 B: plan checks

| # | Check | Severity |
|---|---|---|
| | **Structure, in each state** | |
| B1 | A present bar that is neither attached nor built has a pose in `State.poses`; otherwise it floats at its assembly pose | warning |
| B2 | Every built bar has at least one engaged mate, and the built bars reach the ground through engaged mates | error |
| B3 | Several holders of an unbuilt bar all belong to one robot | error |
| B4 | Every holder's robot has a tool on the bar's part: on the bar or on one of its halves | error |
| B5 | Mates that never become engaged anywhere in the schedule are listed, and halves in no mate (their partner is on a bar the process leaves out) | warning |
| | **Procedure, per movement and across the schedule** | |
| B6 | `ends_on: tools` comes with a tool part (a grip change or a drive); with arm motion it also needs `controller: compliant` | error |
| B7 | An arm motion that moves an attached, unbuilt bar has `grip: closed` on its holders' tools, and several holders move `coupled` | error |
| B8 | A robot that holds a built bar moves only in a compliant movement whose tool part releases that bar | error |
| B9 | A tool on a body its arm does not hold allows only a `linear` retreat, until `on` clears | error |
| B10 | `attached` and `built` change only as the operations allow: a mount or grasp attaches, an insert or a manual step (`ends_on: operator`, a bar placed by hand) builds, an ungrasp releases, an untighten unbuilds | error |
| B11 | Each movement ends where the next one starts (by definition for `target: null`), across actions in `schedule` order | error |
| B12 | A hold is released only after every bar in its `supports_until` is built | error |
| | **Geometry, with robot models** | |
| B13 | `arms` name flange links that end an SRDF group of the acting robot | error |
| B14 | Grasps agree with the geometry. For a tool on a connector half, the grasp is TCP × the half's `pose_in_tcp` × mount offset, and a tool on a half without `pose_in_tcp` is an error; for several holders, forward kinematics agrees whenever joints are known | error |
| B15 | A target's `joints` bring its `link_poses` to their target poses, by forward kinematics, wherever the base and the joints up to the link are known. A miss is a stale solution: the geometry changed after the joints were solved | error |

**Resolved** in the implementation:
- B11 compares joints, grips, tool `on`, present bars, `attached` (holders and grasps) and `built`; bases only within one action
  (a movement that drives a base ends at its target's base),
  because robots drive between actions. Values left open (`null`) are not compared, nor is the acting robot's joints
  after an arm motion whose target gives none.
- B14 compares within 0.1 mm and 1 mrad, against the `pose_in_tcp` of the half the tool is on (§4.3).
  For a built bar, every holder's forward kinematics must give the assembly pose.
- B15 compares within the same 0.1 mm and 1 mrad (inverse kinematics is solved to 0.01 mm). The base is the start
  state's, or the target's where the movement drives it; joints the target leaves out are the start's. `plan_check.unreached_targets` returns the misses as data
  (action, movement, bar, link, distance, angle), and `PlanReport.unreached` carries them, so a process source can
  mark the bars whose solutions are stale. It does not say whether a reached target collides.
- B9 checks that the arm's path is linear; that it moves away is not checked.
- B10 checks within a movement; between movements B11 refuses any change.

## 12. Rationale kept from schema 1

| # | Decision | Reason |
|---|---|---|
| R1 | One world, not one cell per robot | Three robots share one scene; compas_fab's one-robot limit belongs to its adapter (App. A). |
| R2 | One id per object, path-shaped | Paths give grouping without a `kind` field. |
| R3 | `null` for an absent or undecided value | A placeholder value is indistinguishable from a real one. |
| R4 | Full state per movement | Any movement is readable with the process, without replaying earlier ones. |
| R6 | Standard assets by reference | Robots as URDF + SRDF, meshes as files. |
| R7 | Primitives | Bars are cylinders: exact collision, no mesh data. |
| R8 | Copies of robot files in the process | Self-contained and reproducible although calibration changes later. |
| R10 | `schedule` is the only order | Old exports kept order in three places that disagreed. |
| R12 | Attach by links only | A tool is fixed to its flange, so "held by a tool" is held by the flange with the composed grasp. |
| R15 | Schema number plus commit, no conversion | Old processes are read with old code. |
| R17 | Arms named by flange link | Which SRDF group to plan with is a planner choice. |
| R18 | Tool `id` and `kind` both | The id names a geometry variant, the kind the hardware. |
| R20 | Relative robot mesh paths | The process folder can move. |

## Appendix A. Mapping to compas_fab

Planners get a compas_fab cell by syncing a scene (`process.scene_at(movement)`) into a `CompasFabMirror` for the acting
robot (`scaffolding_assembly_core/mirrors/compas_fab.py`). The cell itself is built by `CompasFabCell` (`mirrors/cell.py`), which
viewers also use without PyBullet:

| Process / scene | compas_fab |
|---|---|
| Acting robot URDF + SRDF | `RobotCell.robot_model`, `robot_semantics` |
| Tool of the acting robot | `ToolModel` keyed by its tool id, `frame` = `tcp`; attached to the SRDF group ending at its flange; `touch_links` = mount contacts and the arm's wrist and flange links |
| Other robot | One `ToolModel` of its URDF, its tools welded to the flanges; at its base with its joints, or parked at (50, 50, 0) when absent or when its base or joints are `null` (`mirror.parked`) |
| Body | `RigidBody`; primitives triangulated; `is_hidden` when absent (a half with its bar), and the PyBullet mirrors also move it to (−50, −50, −50) |
| Attached, unbuilt bar and its halves, held by the acting robot | `attached_to_link` = the first holder's link, `attachment_frame` = grasp (× mount offset for a half) |
| Held by another robot, or built | stationary at its world pose |
| Derived contacts (§8.4) | `touch_links` (acting robot links) and `touch_bodies` (bodies, tools, other robots) |

## Appendix B. Old export → schema 2 (`scaffolding_assembly_core.legacy`)

The converter replays the bars' `attached` and `built` flags and the tool states along the schedule; it does not copy
the export's per-state attachments and contacts.

| Old export | Schema 2 |
|---|---|
| `RobotCell.json`, `RobotCell_<Name>.json` | `robots`, `tools` (`tools/<robot>/<name>`; the tool's touch links are not carried), `bodies` |
| `bar_B1`, `env_bar_B1`; `joint_…` | `bars/B1`; `joints/…` |
| Wheel links | `ground_links` |
| `ObstacleRobot<Name>` at `(50, 50, 0)` | `null` |
| Halves carried with a bar in its jointing action | `mount` on each half |
| Half ids | `part`: `T20/Male`, `T20/Female`, `T20/Ground` (every half in the exports is type T20) |
| `J<a>-<b>_male` and `J<a>-<b>_female` | a mate, when both are in the process (4 male halves in each export have no female: their partner sits on a fake bar) |
| `G<n>-…_ground` | a mate with the ground body whose walkable polygons are nearest its assembly position seen from above (0 when above one; the nearest in height wins a tie) |
| Manual mount (`ManualMovement`) | `ends_on: operator`; the bar becomes attached to every flange carrying it or one of its halves (grasps from the export's attachment frames); its target puts the scaffolding tools on the male or ground half at their TCP |
| `grasp` / `close` | grip → `closed`; a support gripper is on its bar from its close, and the bar becomes attached to it (grasp by forward kinematics) |
| Action and movement ids | by role (§3.1): `B10_J_joint` → `B10_J`, its insert → `B10_J_insert` |
| `tighten` (`overlaps_next`) + the insert | one movement, `<action>_insert`: linear, coupled, compliant, `drives: tighten`, `ends_on: tools`; the bar is built at its end |
| `untighten` opening every release | folded into the ungrasp after it: `drives: loosen`, the bar stays built |
| `ungrasp` / `open` | form B: a tool move, grip → `open`, the flange's holders removed; the tools stay on their body until the following retreat, whose target clears `on` |
| A support's `close` | the movement before it (the linear approach) ends with the gripper `on` the bar |
| A tool move setting the grip a tool already has (two support `open`s) | kept: the hardware repeats it |
| Tool poses on halves | `pose_in_tcp` on every `T20/Male` and `T20/Ground` half (`legacy.export.POSE_IN_TCP`) |
| Movement class | `arms`, `path`, `coupled`, `controller` (`joint_tracking` → `position`, `cartesian_compliant` → `compliant`) |
| `lm_axis`, `lm_distance_mm`, `retreat_axes_world` | `line`: the axes and distance, else start to target |
| Notes | per §5.4; planner status dropped |
| Placeholder bar poses | the bar is not present until mounted |
| Robot states (`robot_configuration`, `ObstacleRobot…`) | as in the export |
| `trajectory` | refused (`solutions/`) |
| `WalkableGround.json` (mm) | bodies under `ground/` (m): each walkable polygon extruded 50 mm downward into a slab |
| (none) | `producer.command` = `legacy.export.from_export/<version>`; a converted copy made by another version is converted again |

## Appendix C. Example

`process.json` (abridged):

```json
{
  "format": "husky_process",
  "writer": {"schema": 2.1, "library": "scaffolding_assembly_core", "commit": "3f9c2e71b0ad", "dirty": false},
  "robots": {
    "robots/cindy": {"urdf": "robots/cindy/robot.urdf", "srdf": "robots/cindy/robot.srdf", "files_hash": "9f2c…",
                     "serial": "0806",
                     "tools": {"left_ur_arm_tool0": "tools/cindy/AT3L", "right_ur_arm_tool0": "tools/cindy/AT3R"},
                     "ground_links": ["front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link",
                                      "rear_right_wheel_link"]}
  },
  "tools": {
    "tools/cindy/AT3L": {"collision": [{"mesh": "meshes/AT3L.3e1f0c9a.obj"}], "tcp": [-0.07, 0, 0.08, 0, 0, 0, 1],
                         "kind": "scaffolding_v3"}
  },
  "bodies": {
    "bars/B3":             {"assembly_pose": [0.10, 2.20, 0.05, 0, 0.7071, 0, 0.7071], "collision": [{"cylinder": [0.0125, 0.90]}]},
    "joints/J3-10_female": {"assembly_pose": [0.10, 2.35, 0.05, 0, 0, 0, 1], "collision": [{"mesh": "meshes/T20-Female.a07c55e2.obj"}],
                            "part": "T20/Female", "mount": "bars/B3"},
    "joints/J3-10_male":   {"assembly_pose": [0.10, 2.36, 0.05, 0, 0, 0, 1], "collision": [{"mesh": "meshes/T20-Male.c49d2b80.obj"}],
                            "part": "T20/Male", "mount": "bars/B10", "pose_in_tcp": [0, 0, 0, 0, 0, 0, 1]}
  },
  "mates": [["ground/WG0", "joints/G1-T20Ground-0_ground"], ["joints/J3-10_female", "joints/J3-10_male"]],
  "schedule": ["B1_J", "B1_R", "B3_J", "B3_H", "B3_R"]
}
```

The insertion of B10, one combined movement (abridged):

```json
{"id": "B10_J_insert", "label": "Insert",
 "arms": ["robots/cindy/left_ur_arm_tool0", "robots/cindy/right_ur_arm_tool0"],
 "path": "linear", "coupled": true, "controller": "compliant",
 "line": {"robots/cindy/left_ur_arm_tool0": {"direction": [-1, 0, 0], "distance": 0.015},
          "robots/cindy/right_ur_arm_tool0": {"direction": [-1, 0, 0], "distance": 0.015}},
 "drives": {"tools/cindy/AT3L": "tighten", "tools/cindy/AT3R": "tighten"}, "ends_on": "tools",
 "start": {"robots": {"…": "…"}, "present": ["bars/B1", "bars/B10", "…", "ground/WG0", "obstacles/column_A"],
           "attached": {"bars/B10": [{"to": "robots/cindy/left_ur_arm_tool0", "grasp": ["…"]},
                                     {"to": "robots/cindy/right_ur_arm_tool0", "grasp": ["…"]}]},
           "built": ["bars/B1", "bars/B3", "…"],
           "tools": {"tools/cindy/AT3L": {"grip": "closed", "on": "joints/J3-10_male"},
                     "tools/cindy/AT3R": {"grip": "closed", "on": "joints/J7-10_male"},
                     "tools/alice/SupportGripper": {"grip": "open", "on": null},
                     "tools/belle/SupportGripper": {"grip": "open", "on": null}}},
 "target": {"robots": {"robots/cindy": {"joints": {"left_ur_arm_shoulder_pan_joint": -1.38, "…": "…"},
                                         "link_poses": {"robots/cindy/left_ur_arm_tool0": [0.03, 2.36, 0.13, 0, 1, 0, 0],
                                                        "robots/cindy/right_ur_arm_tool0": ["…"]}}},
            "built": ["bars/B1", "bars/B10", "bars/B3", "…"]}}
```

The grasp, a tool move:

```json
{"id": "B10_J_grasp", "label": "Grasp", "ends_on": "tools", "start": {"…": "…"},
 "target": {"tools": {"tools/cindy/AT3L": {"grip": "closed"}, "tools/cindy/AT3R": {"grip": "closed"}}}}
```

The retreat after the ungrasp, which takes the tools off the halves:

```json
{"id": "B10_R_retreat", "arms": ["…", "…"], "path": "linear", "controller": "position", "line": {"…": "…"},
 "start": {"…": "…"}, "target": {"robots": {"robots/cindy": {"link_poses": {"…": "…"}}}, "tools": {"tools/cindy/AT3L": {"on": null}, "tools/cindy/AT3R": {"on": null}}}}
```
