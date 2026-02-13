# Copilot Chat Session Summary (Bench0+ Findings)

Source: `local_chat_dump/3b1e7614-8225-4945-86dd-b188489f2ee5.json` (extracted transcript: `chatSession-extracted.md`).

## Why this chat happened

Two threads ran in parallel:

1) **Loop-time / performance**: understand where per-loop time was going (especially Hub-to-Hub differences), then create a repeatable benchmark suite (Bench0…Bench5) and use `.rlog` summaries when possible.

2) **PsiKit + Pinpoint logging** (later fully resolved): restore full goBILDA Pinpoint odometry logging under a HardwareMap wrapper while preserving “HardwareMap gating”, avoiding reflection, and keeping AdvantageScope-friendly `/Odometry/<name>` structures.

This summary focuses on the benchmark/loop-time findings plus the most relevant supporting observations.

---

## Early observation: “right_back is slow” (Expansion Hub first-read cost)

- Symptom: drivetrain motors on the **Control Hub** consistently had lower `PsiKit/logTimes`, while the first motor read on the **Expansion Hub** (often `right_back`) looked much slower.
- Interpretation: consistent with **“first Expansion Hub read pays the I/O cost”** (bulk read / bus round-trip), and subsequent reads on that hub benefit from cached/bulk data.

### Bulk caching mode interaction

- PsiKit was using **Lynx bulk caching in `MANUAL`** and clearing bulk cache once per loop.
- TeamCode/Drive code was setting hubs to `AUTO`, effectively fighting PsiKit’s intended caching behavior.
- Fix direction: avoid overriding PsiKit’s bulk caching choice; keep PsiKit’s `MANUAL` behavior in effect.

### “Free with bulk read” vs “extra cost” (conceptual model used)

With `MANUAL` + `clearBulkCache()` once per loop:

- First encoder-related read on a hub triggers one “bulk snapshot” transaction.
- After that, many values are cheap *from the hub*, but there is still per-loop CPU/logging overhead to call getters + format + write log entries.
- Reads like **motor current** tend to be heavier (often non-bulk / extra hub work), and got measured as such in later benches.

---

## Benchmark suite design decisions

- **Bench0**: drive-only baseline (no PsiKit, no Robot) — DS telemetry for loop stats.
- **Bench1**: drive-only + minimal PsiKit session — produces `.rlog` so timing can be analyzed offline.
- **Bench2/Bench3**: getter benches to time specific read blocks (bulk-friendly vs non-bulk/expensive), with later variants **Bench2b/Bench3b** that also start PsiKit so they produce `.rlog`.
- **Bench4/Bench4b**: Limelight overhead isolation (active vs fetched-only).
- **Bench5**: Panels overhead check.

Key principle: **don’t construct `Robot`** in Bench0/Bench1 to avoid extra `hardwareMap.get(...)` calls (keeps baselines clean).

---

## Results: Bench0 baseline (no PsiKit)

From DS telemetry (avg / p95 / max loop dt):

- Typical: **avg ~1.6–1.7ms**
- Typical p95: **~1.6–2.2ms** (another reported p95: **~2.1ms**)
- Max spikes: **~16–20ms**

Interpretation used during the chat:
- p95 is the most useful “driver feel / budget” target.
- max spikes were treated as occasional system hiccups (GC / OS / telemetry cadence).

---

## Results: Bench1 (drive-only + PsiKitMin + `.rlog` writing)

From `.rlog` summary:

- `RealOutputs/Bench/LoopDtMs`
  - avg **4.430ms**
  - p50 **4.052ms**
  - p95 **6.974ms**
  - max **30.419ms**

- `RealOutputs/Bench/PsiKitBeforeUserMs`
  - avg **4.573ms**
  - p50 **4.051ms**
  - p95 **6.909ms**
  - max **1989.802ms** (treated as a rare outlier / storage+GC+OS hiccup)

Breakdown highlights:

- `RealOutputs/PsiKit/sessionTimes (us)/LogOncePerLoopTotal`
  - avg **1404.9us** (~**1.405ms**) and p95 around **~2.0ms**
- `RealOutputs/PsiKit/sessionTimes (us)/DriverStation`
  - avg **423.1us**
- Additional overhead bucket noted: `OpModeControls` around **~0.9ms avg**.

Practical conclusion:
- Bench1 being **~3–4ms slower p95 than Bench0** is expected when a PsiKit session is active and `.rlog` is being produced.

---

## Results: Bench2/Bench3 (initial runs: no `.rlog`)

- Expected behavior clarified: the initial Bench2/Bench3 variants **did not start a PsiKit session**, so **no `.rlog`** is expected.
- DS p95 observations during those runs:
  - Bench2 p95 **~12ms**
  - Bench3 p95 **~14ms**
  - Both increased by a couple ms while driving.

This led to creating `.rlog`-writing variants (Bench2b / Bench3b).

---

## Results: Bench2b vs Bench3b (`.rlog`-based)

### Core stats

- **Bench2b (bulk-backed: pos+velocity for 4 motors)**
  - `RealOutputs/Bench/GetterBlockUs`: p95 **7559us** (avg **6268us**)
  - `RealOutputs/Bench/LoopDtMs`: p95 **16.66ms** (avg **11.90ms**)

- **Bench3b (current reads for 4 motors; treated as heavier/non-bulk)**
  - `RealOutputs/Bench/GetterBlockUs`: p95 **8744us** (avg **7252us**)
  - `RealOutputs/Bench/LoopDtMs`: p95 **17.61ms** (avg **12.78ms**)

Headline comparison:
- Bench3b’s getter block was **~1.2ms slower at p95** than Bench2b.

### Driving vs idle split (no extra robot code)

This split used already-logged joystick axes (`/DriverStation/Joystick0/AxisValues`, indices 0,1,4 for mecanum control).

- Bench2b loop dt p95: `IDLE` **16.07ms** → `DRIVING` **18.46ms** (**+2.39ms**)
- Bench3b loop dt p95: `IDLE` **17.29ms** → `DRIVING` **18.95ms** (**+1.66ms**)
- Bench3b getter block p95: `IDLE` **8637us** → `DRIVING` **9092us** (**+455us**)

Interpretation:
- Driving adds write traffic (motor power writes), increasing hub contention and pushing read blocks higher.

---

## Limelight + Panels quick check (telemetry-only)

User DS-reported p95 ranges (no `.rlog` expected in those versions):

- Bench4b (Limelight fetched-only): **~1.5–2.0ms**
- Bench4 (Limelight active): **~1.8–2.2ms**
- Bench5 (Panels refresh): **~1.8–2.2ms**

Conclusion at the time:
- In that configuration, Limelight + Panels did **not** appear to be big hitters compared to motor getter costs.

---

## Actionable decision: motor current belongs in a “slow tier”

Based on Bench3b being measurably heavier and getting worse while driving:

- Motor current logging should be **rate-limited** (e.g., every 50–100ms), not every loop.

Implementation outcome described in-chat:

- Added optional, throttled motor current logging with:
  - `MotorWrapper.logMotorCurrent` (default `false`)
  - `MotorWrapper.motorCurrentRefreshPeriodSec` (default `0.1` sec)
- Then wired these knobs into a **central tuning file** so TeamCode can keep configuration in one place.

---

## Notes / caveats

- Bench0 vs Bench1 comparisons are most meaningful when run back-to-back on the same hardware/software state.
- Some timing variance (especially very large max outliers) can be dominated by storage/GC/OS/network hiccups.
- Bench2b/3b loop dt includes **both** “getter-block” time and the rest of the loop overhead; the getter block is the cleanest measurement of the incremental read cost.

---

## New measurement: direct `pinpoint.update()` cost (TeleOp)

To stop estimating and measure the actual Pedro call, a timer was added around `super.update()` inside `ThrottledPinpointLocalizer.update()`:

- Metric logged: `RealOutputs/LoggedRobot/UserSectionMS/FollowerUpdate/PinpointUpdate`
- Log file used: `build/psikitReplayOut/MainTeleopPsikit_log_20260104_102003_967.rlog`

From `runRlogSummary`:

- `PinpointUpdate` (ms): avg **6.022**, p50 **5.825**, p95 **7.581**, p99 **9.032**, min **0.626**, max **12.288** (count **600**)

Command used:

```pwsh
.\gradlew.bat :TeamCode:runRlogSummary --no-daemon --rerun-tasks \
  -PpsikitReplayLog="C:\\code\\TeraBridges\\2025_Decode\\build\\psikitReplayOut\\MainTeleopPsikit_log_20260104_102003_967.rlog" \
  -PsummaryPrefixes="RealOutputs/LoggedRobot/UserSectionMS/FollowerUpdate/" \
  -PsummaryKeyContains="PinpointUpdate"
```

Important interpretation:

- This metric is the *direct* cost of Pedro’s `pinpoint.update()` path on-robot.
- It is substantially larger than PsiKit’s `RealOutputs/PsiKit/logTimes (us)/pinpoint` bucket because that bucket measures *logging reads*, not the localizer update path.
