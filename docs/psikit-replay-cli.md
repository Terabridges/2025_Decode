# PsiKit replay from command line (Robolectric)

This repo includes a small command-line runner that can execute FTC `LinearOpMode`s in PsiKit replay mode on your desktop JVM.

## Requirements

- A `.rlog` file on disk
- An OpMode that is **replay-compatible** under Robolectric (typically:
  - uses `FtcLoggingSession` / PsiKit instrumentation, and
  - tolerates `hardwareMap = null` in JVM runs, unless you provide your own stubs)

## Runner

The runner is implemented as a JUnit test:

- [2025_Decode/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/psikit/PsiKitReplayCliRunnerTest.java](../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/psikit/PsiKitReplayCliRunnerTest.java)

It is **skipped by default** unless you pass `-DpsikitReplayOpMode=...`.

## Example command (PowerShell)

```powershell
$logPath = "C:\Users\rbeit\AppData\Local\Google\AndroidStudio2025.2.2\device-explorer\REV Robotics Control Hub v1.0\_\sdcard\FIRST\PsiKit\MainTeleopLogging_log_20251225_221243_356.rlog"

Set-Location "C:\code\TeraBridges\2025_Decode"

.\gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.psikit.PsiKitReplayCliRunnerTest `
  -DpsikitReplayOpMode=org.firstinspires.ftc.teamcode.psikit.ReplayCliSmokeOpMode `
  -DpsikitReplayLog="$logPath" `
  --no-daemon
```

## Notes

- Replay is enabled automatically when `-DpsikitReplayLog=...` (or env `PSIKIT_REPLAY_LOG`) is present.
- During replay, PsiKit disables RLOG server + file output by default.
  - To enable server: `-DpsikitReplayEnableServer=true`
  - To enable writing output logs: `-DpsikitReplayWriteOutput=true`
