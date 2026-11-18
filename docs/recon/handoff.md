# recon HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-18 00:50: recon -- new pilot for the QUDRAC; flies the whole mission but no intact landing yet (d47f98c)

The user (2026-11-17): "I select a point on the map, and the script will fly over that point and
back to the runway as fast as possible"; tune for this one plane; "REMEMBER THERMAL LIMITS ...
running some tests first"; "make sure the pilot doesn't exceed 15gs"; "don't hardcode a speed";
"just make it travel around everytime"; "highest speed where it still has fuel to come home";
"roll as early as possible"; constants only where they make sense as craft tuning.

State: `recon/` (autopilot, guidance, thermal, config, tests, tools/reconfly.py, tools/probe.py),
launcher entry, bundle package. All committed on main, **not pushed**, no howItWorks PDF yet.
Farm: ksp1 only up (start.sh 1). Read `recon/CLAUDE.md`.

Measured (farm, one flight each): probes (logs/probe1, logs/probe-run*): TWR ~7 at Mach 3-4
(thrust 390 kN, 6.4 g axial); cockpit skin the heat limit, 1000->2142 K in 2 s at Mach 4.8 at
5.5 km; heat-limited ~1480 m/s at 20-25 km, ~1370 at 15 km; cockpit interior ~790 K after 300 s.
Missions LOG17748-17761: take-off clean since LOG17747 (40 deg/s rotation at the fin's q); target
pass 300-1550 m at ~130 s; landing: LOG17758 touchdown 80 m/s sink 1.2 cross -3 along 866 then
intake struck; LOG17761 82 m/s cross 0 along 796, intake struck 0.5 s after contact.

**Blocker**: the nose slams down at main-gear contact (intake 0.9 m above ground when level).
Suspect the mains' friction 10 (the user's 2026-09-25 gear rule; also in the craft file) spinning
up below the CoM. Asked the user whether to change friction for this craft.

Next, in order: (1) the touchdown nose slam (friction, or touch down nose-lower); (2) the home
leg's energy plan (1325 s circling the gate in LOG17761); (3) validate the thermal prediction
(kRPC flux units) then fly it; (4) the lap is fuel-impossible on 400 LF -- tell the user;
(5) howItWorks PDF, push.

Traps paid: a wreck left as the active vessel hangs the runway launch (reconfly now loads
qs_warbird first); kRPC-removed waypoints leave map nodes that throw every frame (never remove);
pitch = "present pitch + error" runs away in thin air; climbing on every not-thrust-limited tick
went to 42-55 km; the airplane pilot's hold sagged after lift-off (reverted to as it was);
ground-roll heading departs below ~95 m/s with one wheel unloaded (ROTATE_MIN_Q_PA).
