# PsiKit Replay Tooling Notes (2026-01-17)

This doc describes the **tooling changes** added/modified during debugging of Robolectric replay for `MainTeleopPsikit`, and how to use the tooling to validate replay output `.rlog` files.

## Replay outputs location

Replay-generated logs are written to:

- `2025_Decode/build/psikitReplayOut/`

To find the newest replay output quickly (PowerShell):

```powershell
cd C:\code\TeraBridges\2025_Decode
Get-ChildItem build\psikitReplayOut -Filter *.rlog |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 10 Name,LastWriteTime,Length |
  Format-Table -AutoSize
```

## 1) Robolectric replay runner task

`TeamCode/build.gradle` defines a Robolectric runner task that behaves like a CLI via Gradle properties:

- Task: `:TeamCode:runPsiKitReplayRobolectric`

Recommended PowerShell invocation (quote values to avoid Gradle parsing issues):

```powershell
cd C:\code\TeraBridges\2025_Decode

$log = 'C:\Users\rbeit\AppData\Local\Google\AndroidStudio2025.2.2\device-explorer\REV Robotics Control Hub v1.0\_\sdcard\FIRST\PsiKit\MainTeleopPsikit_log_20260117_124235_238.rlog'

.\gradlew.bat :TeamCode:runPsiKitReplayRobolectric --no-daemon --rerun-tasks `
  -PpsikitReplayOpMode="org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopPsikit" `
  -PpsikitReplayLog="$log" `
  -PpsikitReplayWriteOutput=true `
  -PpsikitReplayOutputDir="C:\code\TeraBridges\2025_Decode\build\psikitReplayOut"
```

What to look for:

- The runner prints `got EOF` and `Replay of log has ended.` when it reached the end of the input log.
- A new `*_Replay_log_*.rlog` file should appear under `build/psikitReplayOut/`.

Log sources:

- **Android Studio Device Explorer** (fastest): replay directly from the Device Explorer cache path (like the example `$log` above).
- **ADB pull**: see PsiKit runbook `docs/ftc-adb-and-rlog-runbook.md` for pulling from `/storage/emulated/0/FIRST/PsiKit`.

## 2) RLOG key dump tooling (strings + targeted keys)

A JUnit helper test exists to decode an `.rlog` and dump key values (including **strings**, like exception stack traces) to a text file:

- Test: `org.firstinspires.ftc.teamcode.psikit.RlogKeyPrinterTest`
- Output file: `2025_Decode/build/psikitReplayOut/rlog-key-dump.txt`

### Why this exists

The normal numeric summary tooling is great for timing stats, but it often **does not surface string values** (e.g., exception messages/stack traces). This test is intended to make replay-init failures deterministic and visible.

### How to run it

This test uses **environment variables** for configuration (reliable under Gradle’s test worker process):

- `PSIKIT_RLOG_DUMP=true` enables the test
- `PSIKIT_RLOG_PATH=...` selects which `.rlog` file to decode
- `PSIKIT_RLOG_KEYS=...` optionally selects a comma-separated list of keys to print values for
- `PSIKIT_RLOG_NEEDLES=...` optionally adds extra comma-separated “substrings to search for” in the key list output

Example (PowerShell):

```powershell
cd C:\code\TeraBridges\2025_Decode

$env:PSIKIT_RLOG_DUMP = 'true'
$env:PSIKIT_RLOG_PATH = 'C:\code\TeraBridges\2025_Decode\build\psikitReplayOut\MainTeleopPsikit_Replay_log_YYYYMMDD_HHMMSS_mmm.rlog'
$env:PSIKIT_RLOG_KEYS = 'ReplayOutputs/ReplayOnly/LoopCount,ReplayOutputs/LoggedRobot/UserSectionMS/ControlsUpdate'
$env:PSIKIT_RLOG_NEEDLES = '__psikit_replay_voltage,Console'

.\gradlew.bat :TeamCode:testDebugUnitTest --no-daemon --tests org.firstinspires.ftc.teamcode.psikit.RlogKeyPrinterTest --rerun-tasks

notepad C:\code\TeraBridges\2025_Decode\build\psikitReplayOut\rlog-key-dump.txt
```

### What the test prints

The dump includes:

- The resolved file path and requested keys
- The number of decoded cycles (`Decoded cycles: ...`)
- Keys matching common debug substrings:
  - `ReplayInitException` (both “in final table” and “anywhere in log”)
  - `ReplayOnly`
  - `UserSectionMS`
  - `Console`
- Requested key values:
  - From the final table
  - And the **last seen value across any cycle** (to capture one-shot keys)

## 3) Replay-init failure markers (consumer-side)

### Problem observed

When `MainTeleopPsikit` fails to construct the full robot during JVM replay, it falls back to a minimal “replay-only” loop.
Historically, exception markers written during the `catch` sometimes did not appear in the replay output `.rlog`.

### Fix

`MainTeleopPsikit` was changed so that replay init exceptions are recorded **from inside** the replay-only loop (after the logging session is reliably active), instead of immediately in the `catch` block.

Files:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleopPsikit.java`

Recorded keys (on fallback):

- `LoggedRobot/ReplayInitExceptionPresent` (numeric)
- `LoggedRobot/ReplayInitExceptionType` (string)
- `LoggedRobot/ReplayInitExceptionMessage` (string)
- `LoggedRobot/ReplayInitExceptionStack` (string)

Note: In PsiKit output logs these show up under `ReplayOutputs/LoggedRobot/...`.

## 4) Replay-only fallback detection

The replay-only loop records a counter:

- `ReplayOnly/LoopCount`

In PsiKit output logs this appears as:

- `ReplayOutputs/ReplayOnly/LoopCount`

Interpretation:

- **Present and large**: the OpMode likely fell back to replay-only mode.
- **Absent**: the OpMode likely initialized normally (no replay-only loop).

## 5) AdvantageScope expectations

What we validate in `.rlog` before opening AdvantageScope:

- `ReplayOutputs/LoggedRobot/UserSectionMS/...` keys exist
- `ReplayOutputs/ReplayOnly/LoopCount` is absent for a “clean replay”
- No `ReplayInitException*` keys exist anywhere in the log
- No `__psikit_replay_voltage` keys exist anywhere in the log (replay-only safety sensor is intentionally hidden from logging)

Note: the exact replay output filename will change on each run.

### Opening in AdvantageScope (Windows)

On this machine, AdvantageScope is installed at:

- `C:\Users\rbeit\AppData\Local\Programs\advantagescope\AdvantageScope.exe`

You can open the replay log by double-clicking the `.rlog` or by launching AdvantageScope with the `.rlog` path as an argument.

Quick UI checklist:

- In the signal tree, expand `ReplayOutputs/LoggedRobot/UserSectionMS/...` and confirm the timing keys exist.
- Confirm `ReplayOutputs/ReplayOnly/LoopCount` does **not** exist (it indicates replay-only fallback).
- Optionally compare against `RealOutputs/LoggedRobot/UserSectionMS/...` for parity.

## Appendix: Known Gradle/test property behavior

In this workspace, `-DpsikitRlogKeys=...` did **not** reach the Gradle test worker JVM as a system property (it showed up as `null`).
Environment variables (`PSIKIT_RLOG_KEYS`, etc.) were reliable.

## Appendix: Summary of changes made during this debug session

### Consumer-side (2025_Decode)

- A Robolectric replay runner task exists and is used for repeatable replay: `:TeamCode:runPsiKitReplayRobolectric`.
- Replay-init failure markers are recorded from *inside* the replay-only loop so the `.rlog` reliably contains strings (type/message/stack) when init fails.
- A JUnit `.rlog` decoder helper was expanded to scan the entire log for “ever seen” keys and to dump string values to `build/psikitReplayOut/rlog-key-dump.txt`.

### Upstream PsiKit changes (drop-in replay hardening)

These were made so replay remains a drop-in replacement and doesn't crash on common SDK assumptions:

- `ReplaySafeTelemetry` wrapper: prevents telemetry `.update()` from throwing during replay.
- `FtcLoggingSession`: installs replay-safe telemetry during replay.
- `HardwareMapWrapper`: ensures a non-empty `VoltageSensor` mapping exists during replay to prevent iterator/lookup crashes.
  - The synthetic voltage sensor name is `__psikit_replay_voltage`.
  - The synthetic sensor is intentionally **not** logged as a device, to avoid confusing extra nodes in AdvantageScope.
- `Logger` console normalization: normalizes console line endings (`\r\n` -> `\n`) to avoid double-spaced console output in some viewers.

If you need the exact source locations, search in the PsiKit repo for `ReplaySafeTelemetry`, `ensureReplayVoltageSensor`, and `normalizedConsoleData`.
