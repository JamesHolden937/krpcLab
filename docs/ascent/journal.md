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
