# Copilot instructions

## Commit messages
- Use **Conventional Commits v1.0.0** for all commit messages.
- Format: `<type>(<scope>): <description>`
  - `type` examples: `fix`, `feat`, `test`, `docs`, `refactor`, `chore`, `build`, `ci`, `perf`, `style`, `revert`
  - `scope` should be a short, lowercase area name (e.g. `teamcode`, `ftc`, `docs`, `build`).
  - `description` should be imperative, present tense, no trailing period.
- If a change is breaking, use `!` (e.g. `feat(teamcode)!: ...`) and include `BREAKING CHANGE:` in the body.

Examples:
- `test(teamcode): add Java PsiKit replay test`
- `docs(teamcode): explain /sdcard/FIRST/PsiKit path`
- `fix(teamcode): avoid port collision with Limelight`
