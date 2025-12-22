# PsiKit PR2 downstream test (AdvantageScope schema)

This doc is for validating the PsiKit "AdvantageScope schema" PR (gamepads + pose) from a real FTC robot codebase using Gradle composite build.

## 1) Point this repo at your local PsiKit checkout

This repo already uses a composite build in `settings.gradle`:

- `includeBuild("../../psilynx/PsiKit")`

So as long as your folder layout matches this workspace:

- `c:\code\psilynx\PsiKit`
- `c:\code\TeraBridges\2025_Decode`

…Gradle will substitute the local PsiKit sources for the published Maven artifacts.

## 2) Checkout the PsiKit PR branch locally

In `c:\code\psilynx\PsiKit`:

- Checkout the PR branch you want to test (ex: `pr/advantagescope-schema`).

## 3) Build

From `c:\code\TeraBridges\2025_Decode`:

- `./gradlew.bat :TeamCode:assembleDebug --no-daemon`

## 4) Run the minimal OpMode

On the Driver Station, run:

- **TeleOp** → **Test** → **PsiKit AdvantageScope Minimal**

Move sticks / press buttons for a few seconds.

Note: the example OpMode uses port **5800** (AdvantageScope default). If 5800 conflicts with another device (e.g., Limelight), change the port in the OpMode and update AdvantageScope to match.

## 5) Find the `.rlog` file

PsiKit’s `RLOGWriter` writes to:

- `/sdcard/FIRST/PsiKit/`

The filename is automatically generated from the OpMode class name:

- `PsiKitAdvantageScopeMinimal_log_<timestamp>.rlog`

Pull the file off the Robot Controller (Android device) and open it in AdvantageScope.

## 6) What to verify in AdvantageScope

- **Joysticks**: gamepad axes/buttons/POV fields show up and update.
- **Odometry**: if a Pinpoint is configured, Pose2d/Pose3d structs appear under `/Odometry/...`.

If there is no Pinpoint device, odometry logs will be absent (expected).
