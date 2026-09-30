# Working Rules

* At the start of every session, load and follow the `caveman` skill when available.
* Use `caveman` to keep communication compact, direct, and technically precise.
* Keep responses concise.
* Do not show full diffs unless requested.
* Before large changes, briefly state the plan.
* When requirements, scope, or decisions are uncertain, ask the user at the start before proceeding.
* Ask upfront only for blocking or high-risk decisions.
* Otherwise, proceed independently using reasonable assumptions.
* Collect non-blocking questions and decisions in a final `Needs your input` section.

## Git Workflow

* Keep the `main` branch stable and runnable at all times.
* Before developing a new feature or making a substantial change, create and switch to a dedicated feature branch.
* Use a clear branch name such as `feature/<name>`, `fix/<name>`, or `refactor/<name>`.
* Make task-related commits on the feature branch as coherent units of work are completed.
* Commit only changes related to the current task.
* Tests, checks, and review are optional; do not treat them as merge prerequisites.
* Merge a completed feature branch into `main` only after the user explicitly confirms.
* Do not develop new features directly on `main`.
* Never push unless explicitly requested.
* Never force-push, rewrite shared history, or delete branches unless explicitly requested.
