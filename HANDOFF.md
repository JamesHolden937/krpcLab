# HANDOFF — read this first, rewrite it last

> **⚠ The farm runs a MODIFIED Kerbol system (MiniSystem, since 2026-11-06), not stock.**
> - Changed: planets' orbits x0.2, Kerbol GM and radius x0.2, moons' orbits x0.5 (Mun at 6 Mm, Minmus at 23.5 Mm).
> - Kerbin's mass, radius, atmosphere and 6 h solar day are stock. Its **sidereal rotation is 21349.47 s**
>   (stock 21549.43) and the year is ~85 days.
> - Kopernicus now actually loads on the farm. Before, it silently didn't, because `KSPTextureLoader` was
>   stripped. It brings its own patches, e.g. solar panels.
> - Farm results from before 2026-11-06 are not comparable with later ones.
> - kspSim models probed before then are stale: re-probe (`kspSim/world.py` warns).
> - The live game (`~/KSP-Retooled`) runs MiniSystem too; KSP-VSE's `vse/` instance is stock.
> - **Since 2026-11-13 (evening) the farm is KSP Retooled 1.13 (Unity 6.7)**, rebuilt from the live game
>   (commit b93d928). Flights before LOG16368 were stock 1.12.5: not comparable. LOG16368 (default booster,
>   first Retooled flight) touched down 31 m/s sideways and was destroyed 203 m from the pad.
> - Install: `testInstances/miniSystem.sh`; back to stock: `testInstances/miniSystem.sh --stock`. Design: `~/minisystem/CLAUDE.md`.


## 2026-11-17 evening: boosterland + ascent -- fin2 from the user's own stack; the burn's divert loses supersonic

State: **kRPC main bc748fd everywhere. Fingerprints unchanged: boosterland a1079928, ascent a68b023d. All committed
and pushed; farm stopped.** howItWorks.pdf (boosterland, ascent) stale. `tools/instancebench.py`,
`docs/krpcFeatureRequests.md` are not ours: leave them. Journal: `docs/boosterland/journal.md` "2026-11-17 evening".

**New saves (d0fac0a):** `qs_stack_fin` (the live Heavy Shuttle craft on the pad, 126 parts, 817 t) and
`qs_finbooster4` (its booster at the farm ascent's separation, LOG17537: 21.2 km, 822/516 m/s, 164.6 t propellant,
staged by the budget with measured drag `4507e5...`). **Fly booster work on `qs_finbooster4` from now on** -- it is
the separation the current ascent code produces; qs_finbooster2 carries a stale, too-light separation.
LOG17538 flew it 14 m / 0.9 m/s with 19.8 t left (budget predicted 28.2 t: 8.4 t optimistic, inside its reserve).

**Ascent fix (3e118b0, flown):** the turn-shape solve counted only upper engines already active or staged with the
decoupler, so the Heavy Shuttle's stage-0 Rhinos gave it 0 kN ("NO ORBIT, kept 0.40"). `Stack.upper_flight` adds the
engines separation lights. LOG17551: solved 0.339, predicted 117.7 t, flew 80 x 80 km with 116.6 t.
**Tell the user:** the craft autostruts its three landing gears Heaviest across the booster's decoupler (the
qs_stack wing-loss setup, fixed there by Grandparent).

**fb4-1117 (LOG17539-17550, 12 x default + DIAG_BURN on qs_finbooster4):** none dry; miss median ~31 m
(4..322, three over 130), first contact median ~6.6 m/s, 1/12 destroyed. Swap 17.2 GB after (restart before
comparing).
- **The coast is fine (2-51 m predicted miss at ignition, all 12); the burn makes the miss.** In the three bad
  flights the predicted miss grows steadily while supersonic (to 190-260 m by Mach 1.1) although the tilt command
  rises; subsonic the tilt pins at `LANDING_MAX_TILT_DEG` (15) and removes only part of it.
- burnplan (plan - flown burn travel): default +86 sd 78 m over ~3.7 km; nose lag -70 m (keep), dead time null,
  isp alone +42 (the wrong way: an unmodelled term). All terms stay (physical; trigger shares reaction_time).

**Next, in order:**
1. **Test the supersonic-divert hypothesis**: `one_law_command` credits a tilt with thrust*sin(tilt) only; the
   tilt's aero force (cla reads -1123 in the burn, fins at the top) may cancel or reverse it at high q. Measure
   from the burn ticks the lateral acceleration per degree of tilt against Mach (DIAG_BURN logs above have them;
   extend burnplan.py), then put the measured aero term (`env.lift_slope_at`, as the coast's aero steer does)
   into the divert's authority and direction -- no fitted constant.
2. Then the burn's stretch: `LANDING_HEADROOM` 0.85, tilt reserve; promote or delete `LANDING_BURN_AIM`.
3. The budget's 8.4 t optimism (LOG17537 v 17538): where between separation and touchdown it goes.
4. Carried: the lag fit swings 0.2-2.6 s in the burn; `LEG_CLEARANCE_MARGIN_M`; the 20 m/s aero-probe floor.

**Traps:** live `.sfs` files are CRLF; strip `\r` before awk. `quickfly.py --instance N` does not set the port;
pass `--rpc-port`. Other projects (`~/kspfarm/koe`) take the farm lock: check `/run/user/1000/farmlaunch-kspfarm.lock`
and wait on it with `flock -w`, never a process-name loop.

## 2026-11-14 evening: ascent -- kRPC 0.8's split MechJeb API (fixed); the farm's thrust cutouts were PersistentThrust (fixed, farm rebuilt)

**The user's live ascent died at liftoff (LOG16691, Heavy Shuttle):** `'AscentAutopilot' object has no attribute
'auto_path'`. kRPC main's KRPC.MechJeb (installed 11-14 with kRPC 0.8) splits the ascent: `ascent_autopilot`
keeps the common settings and `ascent_path_index` (0 classic, **2** PVG), `ascent_path_classic` the turn
(`turn_shape_exponent`, auto path), `ascent_path_pvg` `desired_apoapsis` and the pitch kick. Worse than the
crash: a kRPC object takes any `setattr`, so every split setting had been stored on the Python object and never
reached MechJeb. Fixed (652811c): `MJ_OWNER` routes each setting, `mj_set` refuses a name the object lacks,
`ASCENT_TYPE` 0/1 maps to the path index. Also (4a45566): a pod destroyed after separation ("The active vessel
has no MechJeb") is logged as `upper stage lost`, not a crash. Defaults fingerprint unchanged. Not flown live.

**Farm booster thrust cutouts: found and fixed (farm rebuilt 2026-11-14 ~17:15).** From ~60-70 s every stack's
booster delivered 0 thrust on a growing share of frames (throttle 1, propellant present, no flameout): LOG16692
(qs_stack_small) and LOG16693 (qs_stack) went suborbital. Cause: **PersistentThrust**, whose AllEngines.cfg adds
`PersistentEngine` to every engine. The live game runs the same PersistentThrust at full flow because it also runs
KerbalOverEngineering's `stock/PropellantRequestFix`, and the farm strips KOE (the user, 11-13: take it out of base).
Ruled out first, each by a no-MechJeb vertical flight: MechJeb, krpcLab, kRPC 0.7 v 0.8, GimbalTorqueFix, time scale,
the Retooled build. Fix: the farm rebuilt from today's live build (base Assembly-CSharp = live's; old base and
ksp0-5 in `~/kspfarm/old-retooled-1113/`, safe to delete) and **PersistentThrust added to `testInstances/strip.txt`**
(removed from base and all six clones). After: LOG16694 qs_stack_small 80 x 80, 9.353 t in orbit (solve 9.48);
LOG16695 qs_stack 80 x 80, 64.380 t (solve 65.02). Alternative if the farm should match live more closely: keep
PersistentThrust and install only KOE's PropellantRequestFix (it needs KerbalOverEngineering.Options).
Next: the user's live ascent on the fixed code; boosterland comparisons across the rebuild are not like for like
(new build, no PersistentThrust).

**Trap:** kRPC 0.8 renamed nothing in `Module` beyond what 0.7 did (`common/partmodule.py` covers it), but any
other kRPC class: an unknown attribute *set* is silent. Grep for setattr/assignments on kRPC objects after an API
change, not only reads.

## 2026-11-14 (+ 11-13 night): boosterland -- the boostback oscillation (gimbal lag, the roll latch, gimbalTorqueFix)

**2026-11-14 later: kRPC main (0.8.0-dev, commit 34c70de) installed on the farm, stockref and the live game,
and its client in both venvs** -- `tools/krpcmain.sh` rebuilds/reinstalls it (bazelisk in ~/.local/bin, source
~/krpc); 0.7.0 backups in `.krpc070/` (gitignored). What changed for us: one port (`connect(port=)`; settings
keys `tcpip_address`/`tcpip_port`, `setports.py` writes them; `--stream-port`/`.stream_port` are ignored), every
`krpc.connect` call rewritten, `krpcbatch` on `Session.call`, `fastclient` off (0.7-only: PyPy coast ticks ~30%
dearer, 0.08 -> 0.105 wall-s), launcherMod rebuilt, KRPC.MechJeb now ships with kRPC. The backport stands down
itself ("kRPC models the gimbal lag itself"); boostback identical to it (Kp*alpha 3.29, nose p90 5.7-6.2).
**0.8 also carries #1191: MoI measured about the CoM, ControlSurface torque fixed, `LimitAngularVelocity` on by
default -- a new plant.** k08-1114 (LOG16667-16690, qs_finbooster2, 12 a arm): pace on median miss 480 m, 7/12
destroyed (the coast's held bandwidth reads 0.6 v 1.9 rad/s and kRPC latches 12/12); pace off 26 m, 5/12, td
median 12 m/s (0.7+backport pace on: 66 m, 7.5 m/s). **`ATTITUDE_LAG_PACE` default now False.** kspSim is
broken until ported to the 0.8 protocol (2 suite errors are its 0.7 codec tests).

**2026-11-14: kRPC's own fix (krpc/krpc#1348, merged 11-14, unreleased 0.8.0) backported into
`KrpcGimbalLag.cs` (b53a1d8)**: exact per-tick lag -dt/ln(1-speed dt), cap 2 zeta w0 + w0 <= 1/lag, PI gains
placed for the lagged plant; the patch refuses itself on a kRPC with `gimbalLag` (delete it then). 0.8.0 itself is
not usable early: hashed procedure ids and ~30 other breaking changes need the new Python client and a launcherMod
check; no milestone/date (0.6->0.7 took 91 days). lag4-1114 (LOG16643-16666, qs_finbooster2, same arms as
lag2-1113): boostback lit nose p90 **5.6-6.1 deg/s on all 24** (old patch: trimodal 3.3/10/21), Kp*alpha 3.29
(was 5.33), pace on = off in the boostback; landings 2/12 destroyed pace-on (= lag2), pace-off misses still
+80..+200 E. Burn nose p90 (pace on) median ~5.5 v ~8. On every farm instance; not live. **Trap paid:** the
default `quicksave` (18-part single-Vector booster) is destroyed 24/24 on the Retooled farm with either patch
(lag3-1114, ctlold-1114) -- fly qs_finbooster2 for booster comparisons until that is looked at.

Journal: `docs/boosterland/journal.md` "2026-11-14" and "2026-11-13 night". The user's live LOG16371: the nose swung 55 deg at
full thrust in the boostback (4.3 km of cross). **Two kRPC defects and one stock one, all now handled:**
- **Gimbal lag (kRPC; the user filed an issue).** Auto-tune assumes instant torque; ModuleGimbal Lerps at
  gimbalResponseSpeed (0.125 s, Vector) -> limit cycle under thrust. `ATTITUDE_LAG_PACE` **on by default**
  (rolltune.RollTune.lag_ttp/lag_bandwidth): every phase, from the measured torque-weighted lag.
  Boostback nose p90 15-29 -> 3-25, burn 16-35 -> 4-13 deg/s; coast-entry cross -68..+54 m (was -189..+397).
- **Zero-authority roll latch (kRPC).** No roll authority in the unpowered flip (the fins' RCS can't roll)
  -> auto-tune roll kp 1e4-1e5 -> kRPC latches roll as flexible -> roll at 1/8 gain after ignition, 133-258
  deg/s spin. `KRPC_CLEAR_LATCH_ON_THRUST` **on by default** (Autoland.clear_krpc_latches: AutoPilot.reset +
  configure_autopilot at ignition when latched). Roll p90 133 -> 2 (fin), 173 -> 17 (heavy).
- **Stock ModuleGimbal.GetPotentialTorque over-credits 1.25-1.5x.** `gimbalTorqueFix/` (Harmony mod, build
  with `gimbalTorqueFix/build.sh [--live]`): torque from stock's own GimbalRotation deflection; probe
  measured/credited 0.67-0.80 stock -> 0.85-1.00. **Installed on every farm instance since latch-1113 (the
  farm's plant changed then); NOT in the live game -- the user decides.** Its `KrpcGimbalLag.cs` caps kRPC's
  DoAutoTuneAxis bandwidth by gimbal lag (any kRPC client); lag-1113 showed it never bound (kN m v N m, fixed
  c56c76e); lag2-1113 (fixed): pace off + patch caps kRPC's bandwidth at 5.33 exactly
  as pace does, boostback equally calm in both arms -- **the patch alone fixes the boostback for any kRPC
  client**. Pace is still needed for the burn: from the coast krpcLab sets kRPC's gains itself (auto-tune off),
  so the patch cannot act there (pace off: burn saturated 1.00, fin misses 201-241 m).
- Tools: `testInstances/actuators.py N SAVE --probe` (credited v measured per axis and source);
  `KRPC_AP_DIAG` now also writes LOG<n>.bb.ap.csv (boostback) and .flip.ap.csv (before a latch reset).
  kRPC source: `~/krpc` (git clone, master).
- Deleted after measuring: ROLL_FREE_BOOSTBACK (null), ROLL_TUNE_BOOSTBACK (worked around the latch, hurt
  pitch/yaw). Fixed: holdable probe reading RCS 0 on the valve's reopening tick.

**Open, in order (everything pending as of 2026-11-14 end):**

*kRPC 0.8 (main) -- consequences of the install:*
1. **The 0.8 default rests on an accident.** With pace off, RollTune.engage still turns auto-tune off at coast
   entry but reads pitch/yaw authority 0 at that tick (`held at kRPC's bandwidth - -`, every k08 pace-off
   flight), so the coast and burn fly kRPC's last auto-tuned pitch/yaw gains, unmanaged. Find why accel reads 0
   (0.8's MoI/torque, #1191, or the tick) and decide what the coast should hold -- this replaces old item 4.
2. **Touchdown on 0.8**: median 12 m/s and 5/12 lost v 7.5 m/s, 2/12 on 0.7+backport. Re-take everything
   derived from kRPC torque/MoI (#1191 moved both): RollTune, the probes' tables, `holdable aoa`, flip law.
3. **Second batch, then delete the pace code**: ATTITUDE_LAG_PACE, lag_ttp/lag_bandwidth/pace/unpace,
   ROLL_LAG_FACTOR, ROLL_TUNE_ENGAGE_DELAY_S, and `gimbalTorqueFix/KrpcGimbalLag.cs` (inert under 0.8). Fly
   k08's design again on a fresh farm, plus qs_finbooster and qs_heavy, before deleting.
4. **kspSim is broken**: port `server.py`/`protocol.py`/`fastpb.py`/`wire.py` to 0.8 (Message envelope,
   ConnectionRequest/Response, request ids, one port, streams as uint64, Error.exception, Status ->
   ServerStatus struct), re-record `data/services.bin` (`tools/services.py`), port `attitude.py` to 0.8's
   AttitudeController (#1191 + #1348). Until then the suite has 2 errors (testPhysics wire codec) and the
   post-commit hook will report them.
5. **PyPy fast client**: `kspSim/fastclient.py` stands down under 0.8; coast ticks cost ~30% more wall time
   (0.08 -> 0.105 s). Port it to 0.8's Session (or measure whether CPython+C protobuf is now faster).
6. **Other pilots on 0.8, unflown**: spaceplane, airplane, propPitch (LimitAngularVelocity on by default, surface
   torque and MoI fixed -- their tables and kRPC-dependent laws), ascent (KRPC.MechJeb now ships with kRPC; one
   smoke flight of qs_stack). Fly each pilot's standard batch before trusting its defaults.
7. **Live**: the user flies the 0.8 defaults (latch reset, pace off). START SCRIPT / launcherMod under 0.8 not
   exercised live yet. Live game has no GimbalTorqueFix (stock GetPotentialTorque over-credits 1.25-1.5x):
   offer `gimbalTorqueFix/build.sh --live` (its torque patch still applies under 0.8).
8. **When 0.8.0 is released**: switch `tools/krpcmain.sh` to the release (mod zip + `pip install krpc==0.8.0`),
   diff it against 34c70de, re-fly.
9. **Disk outside krpcLab (the user's call)**: Bazel's cache is 17 GB in `~/.cache/bazel` (`cd ~/krpc && bazel
   clean --expunge` frees it; the next `krpcmain.sh` re-downloads, ~4 min); bazelisk in `~/.local/bin/bazel`;
   `.krpc070/` (50 MB, gitignored) holds the 0.7.0 installs for a rollback.

*Booster, carried:*
10. Why the 18-part `quicksave` booster dies 24/24 on the Retooled farm (lag3/ctlold-1114: parts lost mid-burn,
    ~567 m/s at 5.9 km, either patch). Fly qs_finbooster2 for comparisons until then.
11. Heavy booster's unpowered flip stalls ~120 s -> boostback TIMEOUT, 2-5 km: 4/4 on instance 2, 0 on 5
    (identical GameData and save). Separation roll + no RCS roll authority; find why only ksp2.
12. Flip law reads gimbal authority 0 (gimbal_limit is 0 while locked; cached). Inert today.
13. The landing-burn items of the 2026-11-13 section below (ignition margin, throttle margin, PITCH_YAW_TUNE).
14. Farm memory: six Retooled instances swap 20-30 GB (28.8 GB after k08-1114); consider five.
15. howItWorks.pdf (boosterland) not redone for the latch reset and pace off.

## 2026-11-13: boosterland + ascent -- high-aoa descent refuted and deleted (earlier the same day; still current for the descent, burn and ascent); the fin booster's roll; the ascent's landing budget

**Later the same day (journal "2026-11-13: high-aoa descent", "... the cone"):** item 1b built behind
`HIGH_AOA_DESCENT` (off; defaults fly as before) and flown in 5 batches. **Refuted twice; deleted** (commit after 8aad4ea; code in a0e5f4d), keeping the probe's `lift peak` (polar `aoa:lift/drag`) and `torque limit` log lines.
- Polar (fin boosters): drag/q 7-14 nose-first, 60-85 at 22 deg, 170-375 at 90; lift peaks 45-50 deg, ~0 at 90.
  Torque limit is 90 deg only above ~35 km (q < 400 Pa), 7-15 deg by M2.
- Law 1 (hiaoa1-4, LOG16331-16354, commits 125d60b..e75fdd2): rest broadside, lift on the back branch.
  Commanded 49-70, held 2-42 saturated; burn still lit at 690-720 m/s.
- Law 2 (cone1-1113, LOG16355-16366, commit a0e5f4d, **the code now in the tree**): angle = `holdable_aoa`
  at each propagation step's q; lift direction coned about v at a solved rate/phase (`guidance.solve_high_aoa`,
  `Steer.schedule/cone_rate`); late brakes held while flying. Result vs default, 6 a arm: **ignition speed
  683-747 (median ~701) v 657-754 (~694): no gain; miss median 426 v 44, all long east.** Cause: the cone rate
  is capped by the nose's turn rate (coast flip measured ~9 deg/s, so <= ~0.1 rad/s at 48 deg) and the solve
  pinned it at 0.05-0.09: a minute per revolution, the lift never averages; best miss > 500 m, so the law
  toggled on/off every few s and the coast inherited km-sized errors. At the torque limit there is no torque
  left to move the nose: coning is infeasible on this airframe, not mistuned.
- Deleted: config `HIGH_AOA_*`, the guidance solve, `Steer`'s polar/schedule/cone fields, autoland's high-aoa state and
  brake gate, environment's `drag_nose_first`/`polar_at`, `tests/testHighAoa.py`. Kept: the probe's polar and
  `torque limit` events (`Environment.torque_limit_aoa`, log only).
- **Ascent comparison (the user asked): not flown** -- nothing changes the burn speed, so separation would not
  move. The fin-booster stack on the pad is the live `FMRS/before_launch.sfs` of 11-12 21:49 ("Heavy Shuttle",
  129 parts, all part types present on the farm; untested there): copy it to `saves/` if an ascent needs it.
  `stackfly.py --booster-set` passes booster flags.
- **The user's direction at wrap-up:** asked whether ~5.5 km ignition is high and whether the burn fine-tunes
  touchdown. Facts: LOG16351 lit at 5.8 km / 735 m/s, ~4.2 km of real stop plus ~1.6 km of margin
  (`SUICIDE_MARGIN` 1.18 + `SUICIDE_REACTION_S` 1.5 s), throttle 0.66 at first, 0.95 near the bottom. The burn
  does steer (throttle loop; tilt <= 15 deg + divert, coned upright under 250 m) but on the fin boosters it
  adds error: predicted 1 m at 10 km, 36 m at ignition, landed 71 m short. **Best next lever: the burn's margin
  and its terminal accuracy (item 1a below), not the descent.**
- Farm stopped; swap was 24.3 GB after cone1 (restart before any batch). Defaults fingerprint back to **7cd7531a**.

The user (live LOG16183/16184, a modified fin booster, now `saves/qs_finbooster2`): it oscillates a lot, mostly
roll in the unpowered descent; it ran short of fuel in the burn; it must work with every engine gimballing (they
had turned the outer ring's gimbals down -- not in any save, all 13 flew at 100). Journal:
`docs/boosterland/journal.md` "2026-11-12 night". **Defaults fingerprint 7cd7531a.** All committed and pushed;
**howItWorks.pdf (boosterland, ascent) not redone**. Farm stopped.

**Fuel (ascent, unflown in an ascent):** the separation budget assumed a sea-level terminal-speed landing (405
m/s); the booster lit at 680 m/s and needed ~850. `ascent/budget.fall` falls through the body's density on the
stack's Cd*A table (taken at liftoff, every ascent type) and lights where boosterland's law does: 833 m/s for
that state. `LANDING_CD` deleted. The farm saves start from the old separation and still land at dry mass.

**Roll (defaults now):** stock credits the tail fins with ~100x their roll (kRPC 100-116 rad/s^2 per unit v
fitted 0.75-1.6), so auto-tune's roll loop barely pushed: 70-110 deg/s spins below 20 km.
- `ROLL_TUNE`: from the coast, auto-tune off; roll gains from `common.rateplant.Plant`'s fitted effect
  (`pi_gains`, closing time slowed to the fins' slew, re-read at 1 Hz); pitch/yaw keep the bandwidth kRPC had
  at coast entry (gain x credited accel), dropped to kRPC's oscillation floor while it latches pitch/yaw.
- `ROLL_SYMMETRY`: n from the surface rings (6); n >= 3: roll angle free from the coast, only the rate damped
  (kRPC target roll NaN); n = 2: nearest of 2 roofs.
- Result (attitude4/5-1113, 18 an arm over qs_finbooster/qs_finbooster2): coast/burn roll rate p90 ~0-18 v
  50-105 deg/s, coast nose rate ~1-7 v 5-25; **miss median 51.5 v 39.5** (one 272 m). The fin2 save's own
  6-flight old arms span 28-70.

**Decision (the user, 2026-11-13): keep this controller as the default; fix accuracy later, on top of it.**
Do not revert ROLL_TUNE/ROLL_SYMMETRY to win back the miss.
- `common/rateplant.py`: `Plant`, `actuators`, `closing_time`, `pi_gains`, `authority`, `clamp` moved from
  airplane/control.py (airplane imports them back). launcherMod's `SurfaceActuators` also lists gimbals
  (`gimbalResponseSpeed`, kind 2: lag 1/speed; built and installed in farm + live).
- Tools: `logsum.py --roll` (per phase/4 km: roll error, |r| p90, nose w p90, input, fit); `KRPC_AP_DIAG`
  (kRPC's per-tick record -> LOG<n>.ap.csv, 3000 ticks from the coast or `KRPC_AP_DIAG_BELOW_M`).
- Deleted after measuring: `ATTITUDE_LAG_PACE` (time to peak from actuator lag: boostback cross very
  repeatable but holdable aoa fell to ~12 and the miss 201 v 88), `ROLL_FREE_IN_BURN` (held in the burn 65
  v 44).

**What limits accuracy now** (read off attitude4/5-1113, `logsum.py --segments`, both arms):
- The error is made in the landing burn, not before it. At burn entry most flights are within +-40 m along and
  cross; they land 30-70 m out.
- The burn ends saturated. It lights at ~4-5 km at ~690 m/s; from ~250 m the throttle sits at
  `LANDING_THROTTLE_CAP` 0.95 until touchdown. With no throttle margin left, the horizontal speed is not nulled:
  touchdown hs is 1-22 m/s, median ~8, in both arms. That is 20-50 m of drift in the last seconds, and it is
  the floor both controllers share.
- What the new controller adds: on 5 of 18 flights the nose swings 30-80 deg/s in the burn (old max 38),
  pitch/yaw saturated (`sat=1.00`). LOG16306 (272 m) arrived at the burn 160 m short, then lost pointing
  while it closed. That is the ~12 m median cost and the outliers.

**Open, in order:**
0. **Next (the user chose it, 2026-11-13 end): the landing burn.** Its ignition margin (`SUICIDE_MARGIN` 1.18 +
   `SUICIDE_REACTION_S` 1.5 s = ~1.6 of LOG16351's 5.8 km; throttle opens at 0.66) and why it adds error on the fin
   boosters (predicted 36 m at ignition, landed 71 short; median ~45-90). Restart the farm first (swap 24.3 GB).
1. Accuracy, on the kept controller: (a) leave throttle margin at the end of the burn -- light it earlier, or
   size `SUICIDE_MARGIN` from the craft's thrust/weight -- so the last ~250 m can still steer and null hs;
   (b) the burn's pitch/yaw swings (`PITCH_YAW_TUNE`, item 2, is the candidate); (c) the `holdable aoa`
   probe now runs at whatever roll the booster has (12 v 45 deg between flights).
1b. **High-AOA final descent** (the user, 2026-11-13): first round done, see the top of this section.
   The original notes, kept for context: arrive at the burn slower than ~690 m/s. What the
   fin boosters can hold (36 flights, attitude4/5-1113): the `holdable aoa` probe reads **44-49 deg** on 29
   flights. That is the lift peak (`lift peak at M1.2-1.8: 44-48 deg`, lift/q 135-158 m^2 at 45 deg against
   Cd*A ~15-30 nose-first), so the torque margin is not what binds. On 7 flights, in both arms, it reads 11.5-13.5
   deg: the probe's turn plane against the fin ring's roll, so measure it at the roll the booster flies.
   The coast now flies only 1-5 deg (it only nulls a miss that is already small) and holds 1-3 deg of actual
   aoa below 20 km. Steps: (i) log the probe's drag as well as its lift per angle -- the gain is Cd*A at 30-45
   deg against nose-first; (ii) a law that flies a large aoa for speed, with its lift steered by bank so the
   range still closes (`AERO_STEER`'s propagator has to fly the same law); (iii) on the farm, compare ignition
   speed, burn dv, throttle-cap time and miss against the defaults. Risk: the tracking was 0.1 of a command on
   the original booster; check trk= at the big angles before trusting the model.
2. `PITCH_YAW_TUNE` (branch `pyTune`, unflown): pitch/yaw fitted and capped as roll is.
3. qs_finbooster's boostback roll (|r| p90 115-195, before the coast: gimbals roll there).
4. Fly an ascent on the new budget (live: the user's Heavy Shuttle); re-fly qs_heavy/qs_heavyshuttlebooster
   with the roll tune (airbrakes, no fins: n from the airbrake ring).
5. Carried from 2026-11-12: boostback departure (LOG16157: err 65-77 deg after the flip, held heading took
   cross -2860; gate throttle on pointing error, or re-take the heading); the wing saturating below 9 km after
   a slow flip settle (LOG16045); `DRAG_PROBE_DESCENT_TILT_DEG` 16 stands in for the boostback-exit drag
   (derive it); landing heights by time-to-go (`LANDING_NULL_ALT_M`, `LANDING_DIVERT_MAX_ALT_M`,
   `GEAR_DEPLOY_ALT_M`, `LANDING_UPRIGHT_ALT_M`), `BOOSTBACK_MAX_BURN_S`, `PREDICT_MAX_TIME_S`, `TERRAIN_MAX_M`;
   `BOOSTBACK_TOLERANCE_M` 190 is insensitive over 20-190; re-fly qs_hot/qs_heavy on MiniSystem.

**Traps:** kRPC's diagnostic buffer keeps the *first* 3000 ticks after it is enabled. kRPC resets
time_to_peak after it is set (read it back). With auto-tune off kRPC's mitigation no longer reaches the gains.
The roll rate `r=` has the opposite sense to kRPC's roll input. Swap sat at 7.5-8 GB even after a farm
restart this session. sweep.py must run under `farmlaunch` with its output tee'd to the job's stdout. The
full suite won't run from a git worktree (kspSim tests import the main tree): merge, then run it in main.

## 2026-11-08: airplane -- crash guard deleted; phase margin; test jets; pitch stiffness (fingerprint after `a8e0e27`)

`airplane/` (read `airplane/CLAUDE.md`, "Where it stands" / "Next";
journal 2026-11-08). The user: delete the crash guard from production;
reduce oscillation; work for the vast majority of aircraft; build jets
and screenshot them before flying. All committed; **not pushed;
howItWorks.pdf not redone (crash guard still in it). Farm stopped.**

**Done:** crash guard deleted (`GUARD*`, `pull_up_loss`). Rate-loop tau
from a phase margin (`RATE_PHASE_MARGIN_DEG`, default 32.12 = the old 4
ticks; 47 is the candidate). `control.Stiffness` (pitch input per rad of
alpha, fitted in flight) and `ATTITUDE_TRIM` (off): the trim integrates the
attitude error at g p. `RATE_SETPOINT_WEIGHT` (1 = off). Three stock jets
(`airplane/tools/craftgen.py`, screenshots `logs/shots/craftgen/`),
Seahawk and Suborbital hopper craft in `saves/craft/SPH`.
`airfly --rotate`, runway launch from a loaded flight.

**Unmeasured:** every flag above -- one flight per cell so far. The
trainer's sag after rotation (both margins) is the model's missing
stability spring, which `ATTITUDE_TRIM` targets.

**Next:** re-fly the killed screening batch on **4 instances** (6 put
21-28 GB in swap within two rounds and the reaper killed the batch), then
3+ flights a cell, promote/delete `RATE_PHASE_MARGIN_DEG` 47,
`ATTITUDE_TRIM`, `RATE_SETPOINT_WEIGHT`; PDF; push.

**Traps:** `git commit -a` swept the user's four uncommitted doc edits
(CLAUDE.md, docs/farmSetup.md, docs/testInstances.md,
testInstances/README.md) into a commit once -- undone; they are still
uncommitted, not ours. Commits take ~2 min (post-commit hook). A fresh
instance sits in the space centre and `launch_vessel` hangs there.

## 2026-11-04 late: the window mod is the only UI; the launcher survives scene changes (airplane `570902c5`, propPitch `ca612601`)

The user's request, all done and committed; **nothing here was flown or
run in a game** (the user: "no need to test this"; they test live).

- **`launcherMod` draws everything**: the launcher's menu and every pilot's
  panel (`common/panel.py` is now a client of it: title, telemetry text,
  and controls -- buttons, checkboxes, number fields, choices -- sent as
  `kind|id|label|value` strings and read back by one kRPC stream of
  `PanelRead`). The kRPC-UI panel (`panel.window`, the glyph-drawn frame)
  and `LauncherPanel` are deleted. Without the mod a pilot's panel is
  headless (kspSim; START never pressed, so fly with `--autostart`).
- **Scene changes**: the mod's `EpochWatcher` (an `Instantly` addon kept
  across scenes) counts `onGameSceneLoadRequested` and `onVesselChange`;
  the launcher polls `Epoch()` while a pilot flies and, on a change, stops
  it (SIGINT, SIGKILL after 3 s), takes its panel down and shows the menu.
  The window and toolbar button exist in flight only; the launcher keeps
  running through the space centre / SPH. A pilot's own load or switch is
  announced first with `panel.expect_scene_change(conn, s)` (fmrs: 90 s per
  load; ascent's booster handoff: 10 s).
- **START SCRIPT** (the window with no launcher running): starts kRPC's
  servers (`KRPC.Core.Instance.StartAll`) and runs `env -i HOME=.. PATH=..
  KRPCLAB_GAME_PID=<pid> /bin/bash -lc "cd <root> && exec ./run.sh"`
  (`Z:\usr\bin\env` under Wine; the pid from `/proc/self/stat`).
  `build.sh` writes `GameData/KrpcLabLauncher/PluginData/launch.cfg`
  (`root=`, `home=`). **Unverified**: that Wine's CreateProcess execs a
  Unix binary from Mono's `Process.Start` -- if START SCRIPT says it could
  not run, that is the place; the window shows the exception text.
- **The game's death**: the launcher exits when a kRPC call raises a
  connection error or `KRPCLAB_GAME_PID` is gone, killing the pilot first;
  pilots also get `PR_SET_PDEATHSIG` SIGTERM from the launcher.

**Traps paid:** `tools/bundle.py` only counts identifiers in *git-tracked*
files as reachable from outside a module; an untracked new package's
imports get renamed away ("cannot import name ACCENT_AIRPLANE"). `git add`
a new package before building the bundle.

## 2026-11-04: propPitch -- the prop brake's tip limit; props stopped when still; torque counter deleted (fingerprint `ca612601`)

The tip limit and the stop when still are defaults (below); **the
reaction-torque counter (`TORQUE_COUNTER`, the spin-sense probe, the trims,
the log's `trim=`) is deleted** at the user's request (it read 0.00 on the
only craft it flew). `PropPilot` takes a `gui` so the airplane pilot can run
it. Read `propPitch/CLAUDE.md` and `docs/propPitch/journal.md`.

**What changed (all defaults):**
- `BRAKE_TIP_LIMIT` (`propPitch/tipping.py`): on the wheels, the braking
  thrust is bounded so every wheel stays down with `BRAKE_TIP_MARGIN` 0.5 of
  gravity's restoring moment, from the moment about the foremost/rearmost
  contacts (gravity, measured aero force, the thrust line's height over the
  CoM). Past the balance point the bound is the balance. Hill-climb
  backstop: halves on a contact off the ground 0.25 s or pitch rate > 10
  deg/s, rises 0.25/s; where no bound can be computed it climbs from 0 at
  0.1 g/s. Log `tip=`; events `tip:`.
- Still (|v| < 0.5 m/s): every rotor at 0 rpm with its brake latch set,
  until 2 m/s (`BRAKE_RESUME_SPEED`); `mode=brake-stop`.
- `propfly --wheel-brakes P`; `tools/brakesum.py`.

**Measured (farm, qs_prop_noon):** bound ~-17..-25 kN at low speed, ~0 from
55 m/s. Braked from 33-38 m/s it stopped intact in 8-9 s (LOG15245/15246).
From 55-75 m/s it slides and hops whatever the props do; a plain throttle
cut there breaks up 4/6 (LOG15249-15254): the airframe under the harness.

**Not measured:** the tip limit on a wide wheelbase or props below the
CoM; the brake live.

**Next:** the user flies it live. Then: re-find after a lost part (still
ObjectDestroyedException); a harness that can fly this plane past 55 m/s.

**Traps paid:** "no bound on this side" must never mean "unlimited"
(LOG15225-15230). A bound extrapolated through a filtered acceleration lies
after a 1000 kN thrust swing (LOG15233). The fallback must climb from zero
(LOG15221). A propfly launch of six with an inner `&` returns at once: wait
on the `ROT DONE` marker.

## 2026-11-01: spaceplane -- the approach plans in the middle of each craft's own span; the low-alpha glide is off (fingerprint `07089ff3`)

Committed and pushed; farm stopped (the last batch, heavy-1101, was killed by Claude Code for low memory before it flew -- LOG15091-15095 are part-batch, don't read them). Journal: "Session, 2026-10-31 late" and "2026-11-01". **howItWorks.pdf NOT updated for the approach ratio** -- do it next wrap-up.

**`APPROACH_BEST_LD` is deleted (the user: "remove the hardcoded value").** It was qs_shuttle2's 4.2; the live Heavy Shuttle (best glide 3.0) planned at 4.2 got a 7.2 km gate and landed 765 m short (LOG15006). Now `airframe.approach_ld` = clean reach x sqrt(braked span), at `GATE_ALT_M`:
- clean reach = glide x (h + float)/h: glide is `cla/cda` at 1 g at `APPROACH_FACTOR` x stall, read from the STANDBY sweep **at the dry mass** (`Vessel.dry_mass`, `airframe.stall_at`) and kept on `env.approach_glide_ld`, scaled by `env.approach_ld_scale` (= `hac_ld_scale`, the flown/table ratio, 0.94-1.16 on both craft); float = `(v_app^2 - v_stall^2)/2g`.
- braked span = the split rudder's last `APPROACH_SPLIT_FACTOR` (0.45), or cos(`APPROACH_SCURVE_MAX_DEG`) with no pair.
- Consumers' formulas unchanged (gate, handover `approach_needed`, excess, S-turn, split brake, short divert). The DISAGREES check is gone.
- **qs_shuttle2: 5.11 x 1.23 x 0.67 = 4.22** (gate 7242 m, was 7200) -- inert, by design. Heavy, from LOG15006's live table: L/D 2.65 at 115 m/s, reach 3.37 (flew 3.41), **planned 2.26, gate ~3.3 km**. Unflown on the heavy: see below.
- **Refuted on the way:** planning at the clean reach (bec678c, reverted 6fb047b): reach-1031 0/12, every flight short at 6.3. The table *is* right where flown; the shuttle simply can't be planned with no margin.

**`GLIDE_LOW_ALPHA_SPEED_M_S` 1000 -> 0 (off).** lowa-1101, rigoff 12 an arm: on 2/12 on the runway, **7/12 out of energy 56-82 km short** ("on the ground in HAC"); off 7/12, none. This was last session's unflown default and the user's live flights carried it. The rule says delete or promote: it was the user's idea, so ask before deleting the code.

**`saves/qs_heavyshuttle_rigoff`** (new): the live Heavy Shuttle on rigoff's orbit, LOG15006's propellant (70.37 t), surfaces/gear stowed. Two traps cost two batches: the first copy had full tanks (182.6 t, entry breakup, 0/8 at 100-300 km) and was taken from a persistent saved *after* landing (4 surfaces deployed, gear down: the farm table read CdA 41/ClA 12 at 0 deg against live 25/56). **The stowed version has not flown.**

**Added after the wrap-up (2026-11-01 evening, the user, default, unflown -- they fly it live):** `APPROACH_HOT_BRAKES` -- past the threshold with energy excess (height over best glide to the aim + (v^2-held^2)/2g) > 0, the split rudder goes to its stop and the opposed flaps open at once; latched; still stowed at the flare door. Log line `hot over the runway: every brake out`. Cause: LOG15100 (live Heavy Shuttle) crossed the threshold 460 m up at 95 m/s, rudder 24/38 deg, flaps never opened (they wait for the rudder's stop), rudder retracted at h=400 once the aim was behind it, flare 1.1 km down, touchdown 1726 m along, then an overrun divert north 0.8 s after touchdown on a 2.1 m/s^2 first reading. Also: the overrun divert now waits 1 s after touchdown. Root cause still open: the approach's `exc=` is height-only, and the speed schedule (127 -> 91 m/s) hides ~360 m of energy, so it read 430 m low at the cone exit and opened the brakes 900 m out. Fix candidate: count speed over the held speed in the split brake's factor.

**Next, in order:**
1. Fly heavy-1101: `multirot.sh heavy-1101 2 "qs_heavyshuttle_rigoff|" "qs_stackorb_live1028|"`. Read the `approach at ... planned at` line and the gate (expect ~2.3 / ~3.3 km), and whether the farm table now matches LOG15006's (`aero M=0.3` row). If the heavy still breaks up in the entry, that's its own failure (RCS blocks at 100% skin, then 74 deg alpha at 46 km in LOG15043).
2. Ask the user: delete `GLIDE_LOW_ALPHA_SPEED_M_S`'s code, or find why bank-first runs out of energy.
3. Rigoff off-arm still had 2 ROLLOUT losses and 3 misses (-2449, +1898, -8813) in 12.
4. The brake probe's quiet-inputs version (a3a64cf) still unflown; read `spoiler (measured) ... armed/REFUSED` live.
5. Carried: `APPROACH_SPLIT_FACTOR` is qs_shuttle2's splitprobe table and now also sets the approach's span; LOG14999's 118 m off-centreline touchdown; the stack orbiter's cone outliers; kspSim probe models.

**Traps paid:** a background batch launched from `testInstances/` (the Bash cwd resets there) silently found no `spaceplane/tools` -- `cd /home/holden/krpcLab &&` in every launch. The telemetry's `cla=`/`cda=` are the *table at the commanded alpha*, not flown values. The swept table re-read at the landing mass is noisy (2.4-5.6 best glide on one craft). Zram at 28 GB after two cycles is now normal; restart between batches.

## 2026-10-27: tooling and the farm harnesses; boosterland's farm entry state (bundle 369 kB)

No pilot's flight laws changed.  What did:

- **Bundle 379 -> 369 kB**: C# `name=` no longer pins parameters, test strings
  no longer pin `vec`/`math`, annotations read builtins through aliases,
  `SimpleNamespace`/`Thread` keywords no longer pin parameters.  Post-commit
  hook: its suite bundle was written into the root and failed every commit
  ("came from the tree"); fixed, `post-merge` calls `post-commit`.
- **Dead code deleted**: spaceplane `hac_available`, `airframe.glide_ld`;
  boosterland `landing_attitude`, `terminal_attitude`; `idle.rescan`.
- **Every farm harness flies the bundle** through `tools/farmflight.py`
  (launch in its own process group, reaper, `claim_log` by the pid now in
  every log header); `--tree` flies the modules.  `fly.sh` runs PyPy.
- **From the bundle, `paths.ROOT` was `dist/`**: the launcher's children, the
  spaceplane's logs and the gear cache would have gone into `dist/`.  Fixed
  (a checkout's `dist/` roots in the tree; a copied bundle keeps its own dir).
  Nobody had flown live from the bundle before this fix.
- **quickfly** no longer runs the autoland in-process: the child loads the save
  (`autoland --load`), the instance is held at 1x through the load and the
  child's start-up, the first tick waits for save UT + 4.6 (`--start-ut`), then
  the harness raises the scale.  Every flight engages at 16681.78 +-0.02.

**Boosterland finding (open, not chased):** a save loaded at 1x is a different
booster from one loaded at 4x -- hs 699.6 m/s at 16681.78 every flight, against
701.7-702.8 for the old 4x load -- and on `quicksave` it lands **100-360 m
west** (e1-cpy: 19/64/69/170/259/361 m) where the 4x-loaded state landed
23-155 m, mostly within 60 (e1-old, interleaved on a fresh farm).  Neither the
bundle nor PyPy is the cause (same result under `--cpython`).  The 1x state is
what a live game flies, so the farm booster numbers before today were measured
on a 4x artefact.  Next for the booster: measure over several saves at the new
state; find what a 2.6 m/s entry difference does that the boostback does not
null (the coast miss is ~180 m in both).  Logs: `logs/harness-1027/`.

**Trap paid:** the farm reached 22 GB of swap after an hour of six-instance
booster flights and every comparison flown on it was worthless -- restart
between comparisons, as the farm section says.

## 2026-10-19 night: ascent -- all three stacks reach 80 x 80; the shuttle keeps its wings (fingerprint `5a3c681f`)

**2026-10-21 (farm-wide, see the spaceplane section):** kRPC 0.7.0;
ascent's staging on `Vessel.stage_at`/`decouple_stage_at` (0.7 removed
`Parts.in_stage`/`in_decouple_stage`/`resources_in_decouple_stage`).  One
smoke flight (qs_stack): 81 x 79 km.  KRPC.MechJeb 0.8.1 still loads under
0.7.


All committed and pushed. Farm stopped. The user stopped the session and
will fly the current defaults live themselves (no re-fly asked for).

**Where it stands (defaults):** classic MechJeb, turn shape solved at
liftoff, booster dropped by budget, upper stage followed to orbit.
- qs_stack (shuttle, gears now autostrut Grandparent): 12/12 to 80 x 80,
  **64.48 t sd ~0.1** in orbit (gs1-1019, LOG13070-13081), 0 joint breaks.
  That leaves ~1.0 t LFO = **~51 m/s** (Rhino 340 s) -- the 34 t ore payload
  eats the margin.
- qs_stack_small 12/12 10.17 t; qs_stack_stretch 12/12 10.11 t
  (fix1-1019, LOG13034-13069; Jebediah now in the pod).

**This session changed:**
1. Saves: small/stretch had an uncrewed pod -- no control source after
   separation (all 24 lost). qs_stack gears Heaviest -> Grandparent: the
   struts ran to the booster tank and tore the delta wings off at
   separation (~11/12). Craft file fixed too. **The user's live VAB craft
   still has Heaviest on the three gears** -- told them.
2. Liftoff warnings (no flag, logging only): `upper stage: ... control X`
   / `WARNING upper stage has no control source`; `WARNING autostruts
   cross the booster's decoupler`.
3. `check_settings`: MechJeb ascent settings re-read after separation and
   re-applied if lost (they were not lost in any flight; a guard).
4. turnshape: downrange along the ground (was inertial); the turn starts
   at MechJeb's start speed or height, curve measured from there
   (`Path.start_speed`). **Not flown yet** -- model2-1019 was stopped
   part-way; its partial logs are not a result.
5. `turn shape track:` log line + `ascent/tools/shapetrack.py` (model vs
   telemetry). Before item 4 the model ran 1.4-3.6 km short in downrange
   at separation on all three stacks.
6. Hot staging tried and deleted (did not prevent the wing loss).

**Next, in order:**
1. Whatever the user's live flights report.
2. One batch of the defaults on all three stacks (srot LAPS=2): does the
   turn-start model move the solved shape, and do predicted mass and
   shapetrack agree with the flights?
3. Model gaps still open: drag at zero AoA, booster throttle-down and coast
   drag left out; `RESERVE_FRACTION` per booster; `SEP_ACCEL_RATIO`.

**Traps paid:** `pgrep -f`/`pkill -f` with a pattern that also appears in
your own command line kills the tool's shell (again) -- kill by PID from a
separate listing. The farm restart overwrites every KSP.log: read the
CollisionSpy / explosion lines before restarting. A farm launch with every
kerbal Missing produces an uncrewed pod silently. Swap read 10-15 GB right
after each farm restart tonight (other memory pressure on the machine).

### Earlier spaceplane snapshot -- 2026-10-12 evening: low alpha flown and refuted; the live tumble not reproducible (fingerprint `01de6e69`)

Defaults unchanged all session (every arm was a flag; all deleted).
Committed and pushed to origin/main.  Farm stopped, sleep inhibitor
released.  How-it-works PDF current (the only default change is logging).
Full write-up: `docs/spaceplane/journal.md`, "Session, 2026-10-12 evening".

### Low alpha -- what was found (the user asked for it in the handoff)

The user asked why every plan flies 20-40 deg and whether a lower-alpha
trajectory would be better, thermally checked.

- **Why it is high**: `solve_glide` uses alpha as the primary range control
  (bank is the residual, floored at 30), the deorbit plans inside the box
  (`SOLVE_ALPHA_MIN_DEG` 20, bank 70) to (`ALPHA_MAX_DEG` 32, bank 30), and
  the glide solves in 20-40.  Every plan is high-alpha by construction.
- **Thermal is not the limit.**  Heating rate H = sqrt(rho) v^3 / 1e6 (KSP's
  high-Mach convective coefficient, from Physics.cfg).  The hottest part is
  always the RV-105 RCS block; at v >= 1600 m/s it plateaus at **0.86-0.88
  of its skin limit over H 128-512**, peaks 0.70-0.80 on i90/rigoff and
  0.85-0.88 on e500i45 (H up to 611).  The plateau is not the shock-temp
  ceiling (21 v^0.75 ~ 7000 K), so it is not known past H ~610.  A point
  mass on the logged aero table (calibrated: 220 v 243 flown) gives peak H
  220/257/298/364 at 30/20/15/10 deg, bank 30 -- circular orbits fit inside
  the flown envelope even at 10; e500i45 would exceed it.
- **The shuttle cannot fly low alpha hypersonically by command alone.**
  lowa-1012 (LOG10081-10104; plan 10-20 deg via `SOLVE_ALPHA_MIN_DEG=10` and
  a deorbit band top of 20): **3/12 on the runway v 11/12**, misses to
  -165 km.  At Mach 4-5 it was commanded 16-18 and flew ~22: **its
  hypersonic trim point**.  Fighting the trim leaked into yaw: slip > 10 deg
  on 22% of Mach-4 ticks v 9% on the defaults (LOG10084: 9 commanded, 34
  flown, 28 deg of slip at Mach 3.8).  Heat never bound (skin 0.87 v 0.85).
  The plan also stretched entries to ~2080 km (v ~1500), the shallow regime
  the propagator over-predicts.  Below Mach 2 low alpha is held fine.
- **What would make it work -- move the trim point, then re-plan:**
  1. CoM forward by pumping the nose LFO (`ResourceTransfer`, already used
     as a cone trim) and/or a standing glide trim on the canards/elevons
     (like `CANARD_TRIM` in the cone).
  2. Measure the trimmed alpha by Mach with the broadside probe
     (`BROADSIDE_PROBE_DEG`) before and after.
  3. Only if the trim reaches 15-20 at Mach 4-6, re-fly a lower band --
     and keep the deorbit window out of the >1500 km shallow entries.
  Expected upsides if trimmed: bank changes become body roll (yaw share
  sin(alpha) 57% at 35 deg v 26% at 15), fins out of the wake, brakes
  more effective; downside: higher L/D amplifies model error, more energy
  to dump in the cone, q 2-3x (the low arm's glides reached 12-30 kPa and
  survived).  The code for the band/thermal limits was deleted
  (`git show e145a51`); `deorbit heating` stays in the log.

### The live tumble -- superseded by 2026-10-20 above (aft CoM, `FUEL_TO_NOSE`)

### Added (default, logging only)

- `nz=` load factor on every telemetry line; a `load factor` event at each
  new peak (q, Mach, alpha).  Defaults' glides peak at 1.4-2.4 g.
- `deorbit heating: alpha A peak H, ...` at deorbit commit: each window
  corner's predicted peak heating rate (`trajectory.peak_heating`).

### Where each orbit stands (defaults, this session's farm batches)

tumble-1012 + lowa-1012 defaults: rigoff 12/12, i90 7/8, i90low 3/4 (+ 11/12
in low70-1012, both arms), e500i45 10/12.  Misses: long splashes
(+6.3, +12.7 km), rollout breakups (19-24 parts left), one -2.3 km.

## Next, in order

1. **Landing accuracy on the default band** (the goal: >= 95% stopped,
   0 lost, all orbits): the remaining misses are long overruns after the
   cone (surplus) and rollout breakups.  The handoff of 2026-10-11 item 1
   still applies: measure the split brake by Mach (`splitprobe.py --glide`),
   `HAC_SPLIT_FACTOR` over-credits the upper cone.
2. Low alpha, if wanted: the trim-point measurement above, first.
3. A load-factor guard / destructive probe (the user asked about runtime q
   tolerance): the `load factor` events build the envelope from every log;
   a guard on commanded alpha/bank and a farm-only breaking probe are the
   next steps (not built).
4. `HAC_SHORT_POLAR` (off, unflown) -- re-fly on the current default or
   delete.  Older: `ABORT_NORTH`/`GROUND_SPOILER_ON_NOSE` farm measurement.

## Traps paid this session

- **A farm that won't reproduce a live failure: diff the GameData first.**
  The live install runs ~50 mods the farm does not, one of which feeds a
  joystick into the control state.
- **Lowering the commanded alpha below the airframe's trim is not flying
  lower alpha** -- it is a standing moment the attitude loop holds badly,
  and on this shuttle it couples into yaw.  Move the trim, not the command.
- Batch round length depends on the arm: the low-alpha cycles took 27 min
  v 18 -- estimate from the slowest arm.
- `multirot` arm numbers rotate per cycle: join results to each log's
  `config:` line, never to the `armN` label.
- A source-text test (`TestTheWindowClockIsLoadBearing`) bans the string
  `index == 0` in `deorbit_window`.

## Traps from before (still true)

- Read the save name off each log before comparing.
- A `&&` chain ending in `| tail` commits on a failing test -- check the
  suite's own exit status.
- landsum drops some logs; for outcome counts use the harness lines in
  `logs/rot-*.txt` joined to each log's `config:` line.
- kspSim has no `qs_shuttle2_rigoff` model; its subsonic lift is wrong (gap
  8) -- don't screen cone energy there.
- Six fresh instances put 10-17 GB into zram at boot; 4 cycles reach ~22.
- Kill sims with `pkill -f "kspSim[.]run"`.
- `warp_to` is blocking: the panel does not answer during it.

A global SessionStart hook (`~/.claude/hooks/pushOnStart.sh`) pushes
unpushed commits at session start; read its `pushOnStart:` message.
