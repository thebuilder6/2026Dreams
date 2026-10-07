# Sprint 4 Tickets — Harden + prove (goal: shippable minimal)

Demo: green suite + sim recording of drive/intake/shoot/auto. Total: 5 pts.

---

## S4-1 Smoke tests (2 pts) — depends: S3-4

**Tasks:** create under `src/test/java/frc/robot/` (JUnit 5):
1. `InputShapingTest`: below-deadband ⇒ zero; shaping is odd-symmetric (`f(-x) == -f(x)`); `f(1) == ±1`; 2D angle preserved through deadband+shape.
2. `ZoneGateTest`: Blue `(2,4)` in / `(6,4)` out; Red `(14.5,4)` in / `(10,4)` out (extends S1-3 with pose-level asserts).
3. `ShooterGateTest`: out-of-zone ⇒ no fire; in-zone + slow wheels ⇒ kicker off; in-zone + at-speed ⇒ kicker on; hysteresis (drop 200 RPM keeps firing, drop 800 stops).
4. `ExecutorLifecycleTest`: start ⇒ alive; stop ⇒ thread dead + mission inactive; `Do Nothing` completes without moving.

**Acceptance:** `.\gradlew test --offline --no-daemon` fully green. Single-test shortcut works: `.\gradlew test --offline --tests "frc.robot.YourTestName"`. **Verify:** full suite, twice (daemon-locked runs can flake once — clean re-run is the tiebreaker).

---

## S4-2 Sim validation (2 pts) — depends: S3-4

**Tasks (in `simulateJava`, default GUI, joystick or sim sticks):**
1. Drive 5 m field-relative on Blue then Red; final pose within 0.5 m / 10° of expected; re-zero mid-run and confirm heading snaps.
2. Arm soak: 10× X-toggles; no soft-stop violation, no current-spike warning.
3. Shoot drill: 5× spool-and-fire from 3 m in-zone; every kicker pulse had both errors <150 at onset (check logs).
4. Auto: 3× `Drive2mAndShoot` + 1× `Do Nothing`; record end poses + kicker counts.
5. Replay: open each run's `.wpilog` in AdvantageScope; pose/arm/RPM traces must match the live run.

**Acceptance:** no brownout/overrun console spam (`bind to port 1181 failed` warning is known-benign — ignore); all runs logged; any anomaly filed as a follow-up ticket, not silently fixed. **Verify:** sim runs + notes in sprint review.

---

## S4-3 Docs + backlog (1 pt) — depends: S4-1, S4-2

**Tasks:**
1. Bump `last_verified` stamps touched by this rebuild; append CHANGELOG-style bullets with test evidence (e.g. "S4-1: N/N green on <date>").
2. File deferred-work tickets (do NOT build): vision pose fusion, GameSim fuel/score, multi-bot, Choreo auto paths, Elastic dashboard tab, LED/alert patterns, TestMode/SysId, shooter table re-characterization on real carpet.
3. Update `SPRINT_PLAN.md` status line per sprint (done / carried).

**Acceptance:** reviewer can go from docs alone from empty checkout to green sim; backlog exists for phase 2. **Verify:** docs review by a second pair of eyes.
