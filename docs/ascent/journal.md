# ascent journal

## Session, 2026-10-18 evening: first version

**The user asked:** install kRPC MechJeb on the live game and the farm; write
a simple thing that runs MechJeb ascent guidance but separates when the
booster still has enough propellant to boost back and land; throttle the
booster down before separation so its acceleration is 0.75 of the upper
stage's (they cannot collide), by turning engines off the way the landing
burn does, and confirm it is under before staging. Then: "make sure it's
general; add a fuel tank or something to test."

### MechJeb over kRPC

- Released KRPC.MechJeb 0.7.1 (Genhis, for MechJeb 2.14.3) loads beside
  MechJeb 2.15.3 but logs `MechJeb module AscentAutopilot not found`: 2.15
  split the ascent module. darchambault's fork **v0.8.1** (April 2026,
  "refactored ascent autopilot to support MechJeb 2.15.x") works: flat
  `ascent_autopilot` with `ascent_type` (0 classic, 1 PVG).
- Installed: dll/json/xml in `GameData/kRPC/` of the live game, `base/` and
  hardlinked into ksp0-5; MechJeb2 copied from the live game into `base/`
  and the clones (it was in `strip.txt`; now kept, per-craft settings
  deleted by `mkbase.sh`).
- ksp0 died silently on the first `qs_stack` load (with 0.7.1's init NRE in
  the log); after the swap and a farm restart nothing died again.

### Design

See `ascent/CLAUDE.md`. The separation rule is a model the program computes
from the booster it reads off the staging, not a fitted number:
`budget.recovery`. Its one vehicle constant, the flip rate, was set from the
heavy's log (30 deg/s gives LOG12758's 1251 m/s exactly; 1000 deg/s, an
instant flip, gives 1065).

### Flights

All 1x, defaults, saves `qs_stack` / `qs_stack_small` / `qs_stack_stretch`.

- LOG12760/61 (first): sep 30.8 km, 17.9 km out, 668 away / 674 up, booster
  36.4 t propellant; landed 202 m, standing. Real burns against the model:
  boostback 1045 m/s v 955, landing 347 v 317, coast 1.3 t -- 11.5% over,
  inside the 15% reserve; 3.5 t left. Found: releasing the limits at
  staging let the booster read 2140 kN against a 1280 target -- the limits
  now stay until boosterland's cluster owns them.
- `rot-def1-1018` (18): 7 never launched -- `run_launch` waited on
  `thrust > 0`, MechJeb held the throttle for its countdown, and the next
  try raised on the booster's decoupler. Fixed (`active`, clamps only after
  `LAUNCH_CLAMP_WAIT_S`). The "landed with" readout read the upper stage's
  decouple-stage resources (0 after separation); fixed to the booster's.
- `rot-def2-1018` (18, the fixed code): 18/18 launched and separated, every
  throttle-down confirmed in 0.24-0.26 s at exactly the target (1.51 v 2.02
  g heavy, 0.66 v 0.88 g small). Propellant at first contact (model's
  "left" before the 15% reserve in brackets):
  - heavy 0.58, 3.65, 3.90, 4.17, 5.67, 6.40 t (~7.0): the 0.58 is LOG12796,
    whose landing burn began at 4.6 km from ~770 m/s (the others ~1 km from
    ~270): a boosterland choice, and the reserve covered it.
  - small 0.61-1.37 t (~1.9); stretched 1.41-2.00 t (~2.1). One stretched
    reads -2.69: parts lost before contact (LOG12822, 35 m/s sideways).
  - Landings: heavy 6-358 m, all standing. The small boosters have no legs
    and fall over; two crashed at 57-60 m/s with propellant aboard
    (LOG12798: 130 m/s sideways at 37 m), the stretched one's landing burn
    starts at ~12 km. Those are boosterland on a light single-engine
    booster, not the separation.

### Later the same evening: the flip, and the upper stage

- **Flip law** (the user's idea; boosterland `FLIP_THROTTLE_LAW`, now its
  default): the budget models the same law from the booster's parts.
  `flip1-1018` (36, off v law, n=6 a cell): shuttle stack separates at 32.1
  km with 33.3 t (v 30.7 km, 36.6 t), boostback 20.3 v 23.6 t, landed with
  5.1 v 4.9 t, but lands ~250 m short (coast; boosterland HANDOFF items
  a-c); small 7.1 v 7.6 t boostback, stretched 8.3 v 8.8.
- **HANDOFF=upper never reached orbit.** 900 s timeout too short (now 3600,
  the user). Then: every upper stage burned to depletion -- small stacks to
  hyperbolic with periapsis exactly 80 km, shuttle stack to 460 km x -80
  (later 112 x -22) and fell. (1) PVG's apoapsis target was its default:
  now set (no change). (2) `control.throttle = 1.0` at launch: kRPC holds a
  client-set throttle, so MechJeb could never cut off -- removed; but kRPC's
  throttle read then shows 0.00 through a full-thrust burn and the upper
  stage *still* burned to escape (LOG12922): PVG itself keeps thrusting
  after periapsis passes target (84 km at apoapsis 267 km). Unsolved; PVG
  settings dumped look sane (`optimize_stage_flag` on, `last_stage -1`).
- The throttle-down had been reading `control.throttle` for MechJeb's
  throttle; with the write gone it read 0 and timed out with a 0 kN
  target. Now read from the booster engines' delivered/allowed thrust.

## 2026-10-19: gravity turn and roll

The user: "it won't do a proper gravity turn", and force the roll to the
launch roll (as if launching east), 90 east, 80 km.

- Cause of the steep climb: the script left PVG's pre-guidance pitch program
  at MechJeb's defaults, pitch start 50 m/s and pitch rate 0.5 deg/s (AoA
  limit 5, optimizer only below q 10 kPa -- after separation). Every stack
  flew it lofted: small 55 deg at 10 km / 37 at 30 km, shuttle stack 63 / 46.
- Roll: MechJeb had force_roll on with vertical/turn roll 0, so it rolled the
  vehicle to 0 whatever it sat at. Now set to the pad roll against a
  heading-90 pitch-over. Small stack +89.6 held from liftoff to separation
  (LOG12929); shuttle stack +179.8 held (LOG12935). MechJeb's roll sign
  agrees with kRPC's.
- Pitch rate, small stack, one flight each (trajectory shape, not scatter):
  0.5 -> 80/68/55/44 deg at 2/5/10/20 km, sep 34.1 km (LOG12929, booster
  crashed 333 m out, the usual small-booster landing); 1.0 -> 70/56/45/36,
  sep 30.1 km, booster 10 m from pad (LOG12932); 1.5 -> 63/53/39/28, sep
  26.0 km, 52 m (LOG12931). 1.0 made the default.
- Shuttle stack at 1.0: 63/54/39/25 deg at 2/5/10/20 km, separates 20.5 km
  (was 32 km) with 44.6 t, booster landed 3 m from the pad with 3.25 t
  (LOG12935-36). The heavy stack now drops its booster lower and faster;
  whether that is right for the upper stage is open (HANDOFF=upper is still
  broken). The pitch rate is a fitted constant -- one value for stacks of
  TWR ~1.4-1.7; a kick derived from TWR would be the general form.

## 2026-10-19 evening: follow the upper stage; classic; turn shape solved

The user: follow the ship now that the FMRS button flies the booster; use
classic, not PVG; make the turn shape the most efficient -- computed from
the craft per flight (twice: "you are supposed to calculate it from the
craft", "you shouldn't have to hope it lands somewhere").

- **PVG after a script-staged drop** never re-plans: the upper stage holds
  periapsis 80 km and burns apoapsis to escape. Re-engaging PVG after
  separation: hyperbolic 4/4 (rot-upper1-1019, LOG12943-12950). Classic
  from separation: 80 x 80 on 3/3 (LOG12942, 12949, 12952). Abandoned
  approach: making PVG behave after a manual staging (four attempts over
  two sessions); replaced by classic for the whole flight.
- **Turn-shape sweep** (rot-shape1-1019, qs_stack with the 34 t ore payload,
  6 a cell, farm swapped 15-24 GB): 0.2/0.3 fail 6/6 (flat turn, upper
  stage falls back -- also found HANDOFF=upper flying a wreck to the
  timeout; now ends on "upper stage lost"); 0.4 62.13 t; 0.5 63.00; 0.6
  62.89; 0.7 63.19 (sd ~0.5). Used only as a check of the model, not as a
  value.
- **turnshape.solve** (ascent model with the game's drag table, the same
  separation rule, burn to apoapsis + circularization): LOG12991 solved
  0.524, predicted 64.66 t, flew 64.72 t; predicted no orbit at 0.10/0.28.
  Surplus in orbit ~1.3 t LFO (~70 m/s). LOG12991 sits 1.7 t above the
  sweep's 0.5/0.6 cells -- one flight, unresolved.
