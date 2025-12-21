# Chat Session Recap — 2025-12-21

## Goals
- Make PsiKit FTC wrappers follow the deterministic “snapshot once per loop” contract (no hardware reads in getters; replay uses cached values).
- Ensure `2025_Decode` builds cleanly in Android Studio after moving control code to `GamepadView` + `TelemetrySink` abstractions.
- Clarify developer onboarding requirements while using a Gradle composite build against PsiKit.

## PsiKit (psilynx/PsiKit)
### Wrapper contract work
- Confirmed/locked in the deterministic replay principle:
  - `toLog()` samples hardware once per loop and updates cached fields.
  - `fromLog()` loads cached fields from the log.
  - All getters return cached fields only (no direct reads from `device` and no replay checks needed in getters).

### Key fixes
- Refactored `SparkFunOTOSWrapper` to snapshot semantics:
  - Hardware sampling is performed only inside `toLog()`.
  - All properties now return cached fields rather than `device?.…`.
- Removed the last leftover replay-branch helper (`Logger.isReplay()` usage) from `MotorWrapper` (it was unused).

### Verification
- Built PsiKit FTC module successfully:
  - `:ftc:compileDebugKotlin` => `BUILD SUCCESSFUL` (only deprecation warnings).

## 2025_Decode (TeraBridges/2025_Decode)
### Android Studio compile fixes
Android Studio reported:
- `MainTeleop.java: incompatible types: Robot cannot be converted to Drive`

Root cause:
- `DriveControl` (and other controls) now take subsystem instances (`Drive`, `Intake`, etc.) + `GamepadView`, but `MainTeleop` was still passing the whole `Robot` and raw FTC `Gamepad` objects.

Fix applied in `MainTeleop`:
- Constructed `GamepadView` wrappers via `FtcGamepadView(gamepad1/2)`.
- Passed subsystem instances (`robot.drive`, `robot.intake`, `robot.shooter`, `robot.transfer`, `robot.vision`) into the respective control constructors.

Follow-up compile error:
- `Telemetry cannot be converted to TelemetrySink`

Fix applied:
- Wrapped FTC telemetry as `TelemetrySink` via `new FtcTelemetrySink(telemetry)`.
- Updated `controlsUpdate()` to call `c.addTelemetry(telemetrySink)`.

Verification:
- `:TeamCode:compileDebugJavaWithJavac` => `BUILD SUCCESSFUL` (Java 21 source/target 8 warnings only).

## Design discussion notes
### Why keep `GamepadView` instead of using FTC `Gamepad` everywhere?
- The FTC `Gamepad` type is strongly tied to the FTC runtime environment.
- `GamepadView` keeps control logic desktop-safe so it can be driven by:
  - FTC `Gamepad` on-robot (via `FtcGamepadView`)
  - replay/test harnesses on desktop (via custom implementations like `ReplayGamepad`)

### Composite build onboarding (current state)
- `2025_Decode/settings.gradle` uses `includeBuild("../../psilynx/PsiKit")`:
  - for the Gradle plugin `org.psilynx.psikit.buildinfo`
  - and to resolve `org.psilynx.psikit:core` / `org.psilynx.psikit:ftc` via dependency substitution.

Implication for new devs (while composite build remains):
- They need PsiKit available locally (typically by cloning both repos) and the `includeBuild` path must be valid on their machine.
- They do **not** need `publishToMavenLocal` for PsiKit; the composite build can build from source.

## Next steps (optional)
- Once PsiKit artifacts are published to the hosted repo, switch off the composite build by removing `includeBuild(...)` and relying on the hosted coordinates.
