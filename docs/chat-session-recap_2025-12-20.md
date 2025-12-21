# Chat Session Recap (2025-12-20)

This file summarizes the work performed in this chat session across the multi-root VS Code workspace:

- `c:\code\TeraBridges\LogTest`
- `c:\code\TeraBridges\2025_Decode`
- `c:\code\psilynx\PsiKit`

It is written as an engineering handoff: what we changed, why, current status, and what’s still open.

---

## 1) Original goal(s)

### A) Build metadata visibility in AdvantageScope
Goal: have build metadata visible under `RealOutputs/PsiKit/BuildInfo/*` and have Git fields populated (not `unknown`).

Observed issues at the start:
- `RobotConfigName` looked like a JSON-ish string rather than a clean value.
- Git fields (SHA, branch, etc.) were coming through as `unknown`.

### B) Reduce consumer Gradle boilerplate
Goal: make `TeamCode/build.gradle` as lean as possible and upstream build-info generation into PsiKit so FTC projects just apply a plugin.

### C) Align with AdvantageKit-style (gversion-like) build constants
Goal: match the “way AK does it” (fields like `DIRTY`, `GIT_REVISION`, etc.) while staying configuration-cache safe in FTC/Android Gradle builds.

### D) Desktop replay/testing + IO pattern
Later pivot: running the full FTC OpMode on desktop is not realistic, but replaying logic on desktop is useful and achievable.
Goal: start refactoring toward an IO pattern so key logic can run on desktop with `.rlog` replay.

---

## 2) PsiKit-side work (upstream)

### A) Build info generation moved to a PsiKit Gradle plugin
Work was upstreamed into PsiKit so consumers don’t copy custom tasks.

Key elements:
- A PsiKit Gradle plugin generates a Java file `PsiKitBuildInfo.java` as a generated source directory.
- The generator emits AdvantageKit/gversion-like fields (e.g., dirty as an `int`, revision, date).
- The implementation was kept configuration-cache compatible.

### B) Git fields were “unknown” → fixed
Root cause: the generated build constants file literally contained `"unknown"` because the generator’s git invocation wasn’t running (or wasn’t safe) under configuration cache.

Fix approach:
- Changed the generator’s git invocation to a configuration-cache-safe approach.
- Ensured incremental behavior by declaring `.git` as an input (so rebuild triggers when Git state changes).

Result:
- Generated `PsiKitBuildInfo.java` contained correct Git info (SHA/branch/date/dirty) when checked.

### C) RobotConfigName normalization
`RobotConfigName` was normalized upstream (extracting `"name"` from a JSON-like value rather than logging the whole structure).

### D) TeamNumber logging was attempted then removed
Team number capture/mirroring was explored, but per user request (“remove it all”), TeamNumber logging was removed.

### E) Recording BuildInfo upstream inside PsiKit FTC session
Downstream OpModes had been manually calling `psiKit.recordMetadata(...)` with BuildInfo fields.
We moved this responsibility upstream into `FtcLoggingSession.start()`.

### F) Runtime “BuildInfo missing” troubleshooting
After upstreaming BuildInfo recording, some BuildInfo values did not appear in AdvantageScope on-robot.
We attempted a reflection/classloader approach to find the generated `PsiKitBuildInfo` class on FTC runtime (including trying multiple classloaders: OpMode, thread context, PsiKit classloader).

Status:
- Still not proven working on-robot; user pivoted focus to desktop replay / IO pattern.

---

## 3) Consumer-side work in 2025_Decode

### A) Keep TeamCode Gradle lean
`2025_Decode/TeamCode/build.gradle` was kept minimal and just applies the PsiKit build-info plugin:

- `plugins { id 'org.psilynx.psikit.buildinfo' }`

### B) Remove manual BuildInfo metadata recording from OpModes
In `2025_Decode`, manual BuildInfo `Logger.recordMetadata(...)` calls were removed from:
- `TeamCode/src/main/java/.../opmodes/teleop/MainTeleopLogging.java`
- `TeamCode/src/main/java/.../opmodes/autonomous/MainAutoLogging.java`

These OpModes now rely on PsiKit’s upstream session start behavior.

### C) `.gitignore` updated for logs
`2025_Decode/.gitignore` was updated to ignore `*.rlog` log files.

---

## 4) Desktop tooling (PsiKit tools)

To support desktop verification/debugging:
- A multi-command entry point was added in PsiKit tools.
- A log inspection command was added to print metadata and BuildInfo outputs from `.rlog` files.

Purpose:
- Quickly validate BuildInfo/metadata presence without needing to run on-robot.

---

## 5) IO-pattern refactor (2025_Decode TeamCode)

We investigated the TeamCode architecture to assess replay feasibility.
Findings:
- Subsystems like `Drive`, `Intake`, `Shooter`, `Transfer`, `Vision` directly depend on FTC SDK classes and call hardware APIs inside `update()`.
- Controls contain mostly decision logic, but still depend on FTC `Gamepad` and `Telemetry`.

### Implemented: Drive IO abstraction (first step)
We refactored only Drive (smallest/safest subsystem) into an IO pattern:

Added:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystems/io/DriveIO.java`
  - `DriveIO.Inputs` snapshot
  - `updateInputs(inputs)` and `setMotorPowers(lf, rf, lb, rb)`

Added FTC implementation:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystems/io/DriveIOFtc.java`
  - Holds `DcMotorEx` instances and implements `DriveIO`

Refactored:
- `TeamCode/src/main/java/.../config/subsystems/Drive.java`
  - No longer references FTC SDK types.
  - Owns `DriveIO` and calls `io.updateInputs(...)` then `io.setMotorPowers(...)`.

Wired:
- `TeamCode/src/main/java/.../config/subsystems/Robot.java`
  - `drive = new Drive(new DriveIOFtc(hardwareMap));`

### Fixed compilation breakages in test OpModes
Some test OpModes instantiated `new Drive(hardwareMap)` and/or accessed `drive.leftFront` directly.
Updated:
- `LockAutoTester`, `LockTester`: build Drive via `new Drive(new DriveIOFtc(hardwareMap))`.
- `transferTester`: switched from direct motor access to `drive.setDrivePowers(...)` + `drive.update()`.

### Build verification
- `:TeamCode:assembleDebug` succeeded after updates.

---

## 6) Desktop replay harness in 2025_Decode (sandboxlib)

Goal: prove the IO approach can run desktop-side.

Added a minimal desktop main:
- `2025_Decode/sandboxlib/src/main/java/org/firstinspires/ftc/teamcode/desktop/DriveReplayMain.java`
  - Loads `.rlog` using `org.psilynx.psikit.core.rlog.RLOGReplay`.
  - Steps `Drive` using a stub `DriveIO` that prints requested motor powers.

Gradle wiring:
- `2025_Decode/sandboxlib/build.gradle`
  - Adds `org.psilynx.psikit:core:0.1.0-beta2` as a dependency.
  - Adds task `runDriveReplay`.
  - IMPORTANT: because `:TeamCode` is an Android module, `sandboxlib` does **not** depend on `project(':TeamCode')`.
    Instead, `sandboxlib` compiles only the desktop-safe TeamCode sources via `sourceSets` includes:
    - `Subsystem.java`
    - `Drive.java`
    - `DriveIO.java`
    (explicitly excludes FTC-only `DriveIOFtc.java`)

Composite build wiring:
- `2025_Decode/settings.gradle`
  - Added `includeBuild("../../psilynx/PsiKit")` for dependency resolution (not only pluginManagement), so `org.psilynx.psikit:core:0.1.0-beta2` resolves from the local PsiKit repo.

Build verification:
- `:sandboxlib:compileJava` succeeded.

---

## 7) Known IDE/classpath issue in VS Code

Symptom:
- `DriveIO.java` sometimes appears “not on the classpath” in VS Code (only syntax errors are reported).

Reality:
- Gradle build succeeds and compiles `DriveIO.java` as part of `:TeamCode`.

Likely cause:
- VS Code Java/Gradle/Android project import did not fully load the Android module in a multi-root workspace, so Java language services treat some files as loose/unimported.

Mitigation:
- Ensure the Gradle root is imported.
- Use Gradle refresh.
- Use Java language server clean (noting that it may restart VS Code).

---

## 8) Current status / next steps

Completed:
- PsiKit build-info generation plugin (config-cache safe) and Git metadata correctness.
- Consumer Gradle kept lean.
- Drive refactored to IO pattern and still builds on FTC.
- Desktop harness compiles and can replay `.rlog` and step desktop-safe logic.

Open / not yet fully resolved:
- On-robot BuildInfo reflection-based recording still not confirmed working in AdvantageScope.
- Desktop replay harness currently does not map real logged driver inputs into `Drive.setDrivePowers(...)` (it only steps `Drive.update()` and prints outputs).

Recommended next incremental step:
- Identify the log keys for joystick axes or computed wheel powers and wire `DriveReplayMain` to pull them from `LogTable` and call `drive.setDrivePowers(...)` each tick.
- Then repeat the IO extraction pattern for the next subsystem with minimal dependencies (likely `Transfer` or `Intake`), leaving `Vision` (Limelight SDK) for later.
