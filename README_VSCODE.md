# VS Code quick build buttons (2025_Decode)

This repo already has VS Code tasks wired up for Sloth + Gradle.

## Fastest buttons (status bar)

1. Install the recommended extension: **Task Buttons** (`spencerwmiles.vscode-task-buttons`).
   - VS Code should prompt you automatically (based on `.vscode/extensions.json`).
2. Reload VS Code window (Command Palette → **Developer: Reload Window**).
3. You should see status-bar buttons:
   - **Sloth Deploy** → runs `:TeamCode:deploySloth`
   - **Sloth Reset+Deploy** → runs `:TeamCode:removeSlothRemote` then `:TeamCode:deploySloth`
   - **Builds** → quick-pick menu for common Gradle tasks

### Multi-root workspace note (important)

If you have multiple folders open in one VS Code window (multi-root workspace), the Task Buttons extension reads **workspace** settings (from your `.code-workspace` file), not each folder’s `.vscode/settings.json`.

In that setup, copy the `VsCodeTaskButtons.*` settings from [2025_Decode/.vscode/settings.json](.vscode/settings.json) into your workspace file’s `"settings"` block.

## Built-in options (no extensions)

- **Ctrl+Shift+B** → runs the default build task (currently Sloth deploy).
- Command Palette → **Tasks: Run Task** → pick one of the Sloth/Gradle tasks.

## Where tasks live + default build task

- Task definitions: [TeamCode tasks](.vscode/tasks.json)
- Default build task: the task whose `group` has `"kind": "build"` and `"isDefault": true`.
- Change it via Command Palette → **Tasks: Configure Default Build Task** (or edit `.vscode/tasks.json`).

## If deploying over Wi‑Fi ADB

- Connect once: `adb connect 192.168.43.1:5555`
- Then run Sloth deploy.
- Optional targeting: set `ANDROID_SERIAL=192.168.43.1:5555` in your environment.
