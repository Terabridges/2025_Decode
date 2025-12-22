# PsiKit PR Plan (Draft)

## Goals
- Prepare PsiKit changes as small, reviewable PRs.
- Start with two PRs:
  1) Fix stability of `RLOGServer`
  2) Align logged Gamepad + Pinpoint odometry (Pose2d/Pose3d) data for AdvantageScope visualizations

## Constraints / Preferences
- Keep the Gradle composite build approach for now (consumer repo includes PsiKit via `includeBuild`).
- Prefer PRs that are logically independent and easy to review.
- Avoid “stacked” PRs unless there is a hard dependency.

## Recommended PR Breakdown

### PR 1 — RLOGServer Stability
**Branch name:** `pr-rlogserver-stability`

**Intent:** eliminate RC-app crashes / races when stopping logging while `RLOGServer` is/was active.

**Primary files (expected):**
- `core/src/main/java/org/psilynx/psikit/core/rlog/RLOGServer.java`
- `ftc/src/main/java/org/psilynx/psikit/ftc/FtcLoggingSession.kt` (only if needed to adjust receiver lifecycle / add guards)

**Acceptance:**
- Does not crash when the OpMode stops and logging ends.
- Server lifecycle is idempotent (`start()` safe to call twice; `end()` safe to call twice).
- Threading/race safety: no TOCTOU on `thread/server` references.
- Build check: `:ftc:compileDebugKotlin` succeeds.

### PR 2 — AdvantageScope Schema: DriverStation + Pinpoint Odometry
**Branch name:** `pr-advantagescope-schema-gamepad-pose`

**Intent:** Ensure logged inputs/poses are shaped exactly as AdvantageScope expects.

**Primary files (expected):**
- Gamepad (Joysticks schema):
  - `ftc/src/main/java/org/psilynx/psikit/ftc/DriverStationLogger.kt`
  - `ftc/src/main/java/org/psilynx/psikit/ftc/wrappers/GamepadWrapper.kt`
- Pinpoint odometry (Pose2d/Pose3d structs under `/Odometry`):
  - `ftc/src/main/java/org/psilynx/psikit/ftc/PinpointOdometryLogger.kt`
  - `ftc/src/main/java/org/psilynx/psikit/ftc/StructPoseInputs.kt`
  - `ftc/src/main/java/org/psilynx/psikit/ftc/FtcLoggingSession.kt` (calls these loggers)

**Acceptance:**
- AdvantageScope can plot joysticks using `/DriverStation/Joystick0` and `/DriverStation/Joystick1`.
- AdvantageScope can select and plot Pose2d/Pose3d struct values under `/Odometry/...`.
- Logs do not contain conflicting legacy keys for pose fields (legacy subkeys removed).
- Build check: `:ftc:compileDebugKotlin` succeeds.

## How to Split Work from a Single “Everything” Branch (few commits)

### Preferred approach (no history rewriting): new PR branches from `main` + selective cherry-pick
1) Create a safety backup branch pointer:
- `git branch backup/all-changes`

2) Create PR1 branch from clean `main`:
- `git checkout main && git pull`
- `git checkout -b pr-rlogserver-stability`

3) Bring over changes without committing (so we can stage only what belongs):
- `git cherry-pick -n <sha...>`

4) Stage only the PR1 files and commit:
- `git add core/src/main/java/org/psilynx/psikit/core/rlog/RLOGServer.java`
- (optionally) `git add ftc/src/main/java/org/psilynx/psikit/ftc/FtcLoggingSession.kt`
- `git commit -m "Fix RLOGServer shutdown stability"`

5) Discard accidentally-applied unrelated changes:
- `git restore --staged .`
- `git restore .`

6) Repeat steps 2–5 for PR2 (`pr-advantagescope-schema-gamepad-pose`).

### Alternative (more git surgery): interactive rebase and split commits
- `git rebase -i main`
- mark mixed commits as `edit`
- split via `git reset HEAD^`, then `git add` subsets + commit, then `git rebase --continue`

## Verification Commands
- Compare your branch to main (for scope clarity):
  - `git diff --name-status main...HEAD -- ftc/src/main/java/org/psilynx/psikit/ftc/PinpointOdometryLogger.kt ftc/src/main/java/org/psilynx/psikit/ftc/StructPoseInputs.kt ftc/src/main/java/org/psilynx/psikit/ftc/FtcLoggingSession.kt`
- Build:
  - `./gradlew :ftc:compileDebugKotlin --no-daemon`

## Notes
- If PR2 turns out to already be on `main`, skip PR2 and focus on deltas only.
- If RLOGServer stability changes are purely internal to `core`, keep the FTC layer unchanged to minimize blast radius.
