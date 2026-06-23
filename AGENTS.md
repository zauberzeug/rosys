# AI Agent Guidelines for RoSys

This file gives AI assistants project-specific behavioral guidance for working on RoSys.
For human-facing contribution conventions, see [CONTRIBUTING.md](CONTRIBUTING.md).
For what RoSys is and how the pieces fit together, start with [README.md](README.md) and the [documentation](https://rosys.io/).

## Orient before you touch code

RoSys is a robotics framework, not a typical web app.
A few facts shape almost everything:

- **Single asyncio event loop, built on NiceGUI.**
  Robot logic runs cooperatively on one loop — blocking it stalls the whole robot.
  Heavy work goes through `rosys.run.cpu_bound`/`io_bound`, and a few subsystems run their own process (e.g. path planning's `PlannerProcess`).
  Any multiprocessing must use the `spawn` start method: RoSys sets it under `if __name__ == '__main__'` and **asserts** it at startup, so an imported app with the wrong method fails fast rather than being auto-corrected.
- **Two brains.**
  Time-critical and safety-critical logic (motor control, E-stop, bump-stop) lives on the microcontroller as [Lizard](https://lizard.dev/) code (the "Robot Brain"); Python orchestrates and reacts.
  Python requests, the microcontroller decides.
- **Simulation-first.** Most robot-facing modules have a simulation twin, so you develop and test against simulation with accelerated time and no physical robot.
  (Pure support modules like `CanHardware`, `SerialHardware` and `ExpanderHardware` are hardware-only — no twin.)
  Make it work in simulation before you reach for hardware.

## Pair programming

- **Research before guessing.**
  This codebase is conventions-first — grep for a similar module and copy its shape rather than inventing a new one.
- **Discuss trade-offs** when the design is unclear instead of silently picking one, and break large changes into reviewable steps.

## RoSys golden rules (easy to get wrong)

- **Time:** for RoSys-time delays use `await rosys.sleep(seconds)` and `rosys.time()` — they honor simulated/accelerated time so tests can fast-forward.
  Never `time.sleep()`, and don't use `asyncio.sleep()` for robot-time waits.
  (`asyncio.sleep()` is fine for plain event-loop yielding or wall-clock polling, as RoSys itself does in path planning and automation.)
  `rosys.set_time()` is tests-only.
- **Lifecycle:** register with `rosys.on_startup` / `rosys.on_shutdown` / `rosys.on_repeat`, **not** NiceGUI's `app.on_startup`.
  `rosys.on_startup` runs _after_ persisted state is restored (each `Persistable` restores itself when `.persistent()` is called), so configuring through `app.on_startup` lets the restored state overwrite whatever you set (see the README note).
  Note `on_repeat` waits one interval before its first call — it is not immediate.
- **Don't block the loop:** wrap CPU-heavy work in `await rosys.run.cpu_bound(...)` and blocking I/O in `await rosys.run.io_bound(...)`.
  Both return `R | None` — `None` when the app is shutting down or the call is cancelled, so guard the result.
  `cpu_bound` runs in a _spawned_ subprocess, so its callback and arguments must be picklable (module-level functions, no closures or `self`).
- **Events** are typed `Event[T]` with `.emit()` / `.subscribe()` (imported from `nicegui`, not defined in `rosys` — grepping the `rosys` package for `class Event` finds nothing); event names are `SCREAMING_SNAKE_CASE` (e.g. `VELOCITY_MEASURED`).

## Hardware modules: keep the Hardware/Simulation pair intact

A robot-facing hardware subsystem is usually a triplet — keep the symmetry so simulation and tests keep working:

1. an **abstract base** defining the interface and events,
2. a **`*Hardware`** class that talks to the Robot Brain (emits Lizard code, parses sensor data in `handle_core_output`), and
3. a **`*Simulation`** class that updates state in `step(dt)` without any hardware.

Use an existing module such as `rosys/hardware/wheels.py` or `rosys/hardware/bumper.py` as the blueprint, and export all three.
Note this is a _convention_, not universal: pure support modules (`Can`, `Serial`, `Expander`) are hardware-only.
`RobotSimulation` casts its modules to `ModuleSimulation`, so don't add a hardware-only module to a simulated robot.

## Safety boundary — do not casually cross

- Safety- and timing-critical actuator logic belongs in Lizard on the microcontroller, which continuously enforces the safety conditions and performs the time-critical control.
  Python may request actions via `robot_brain.send(...)`, but must not bypass the microcontroller or drive hardware directly.
- The Lizard _startup script_ is assembled from each module's `lizard_code` by `RobotHardware.generate_lizard_code()` and uploaded via `robot_brain.configure()`, which restarts the brain to apply it.
  This is config/startup-script upload, **not** firmware flashing (that is `lizard_firmware.flash_core()` / `flash_p0()`).
  Don't expect to live-tweak it; changing module composition means regenerating and re-configuring.

## Testing

- pytest with `pytest-asyncio` in `asyncio_mode = "auto"` — tests are plain `async def`, no decorator needed.
- Develop against the `*Simulation` classes and the helpers in `rosys.testing`: `forward(seconds=...)` to advance accelerated time, or `forward(x=..., y=...)` / `forward(until=...)` to run until the robot arrives or a condition holds, plus `assert_pose(...)` and `approx(...)` (a deep dataclass-equality helper, not `pytest.approx`).
  See `tests/test_robot.py` and `tests/test_time.py` for the idiom.
- Add tests for new functionality.
  Run a single test with `uv run pytest tests/test_<x>.py::test_<y>`.

## Before you call it done

Run the same gates CI runs (`.github/workflows/pytest.yml` is the source of truth):

```bash
uv run pre-commit run --all-files       # ruff --fix (+ import sort), autopep8 (120 cols), single quotes, codespell, mdformat
uv run mypy ./rosys                      # not part of pre-commit — run it explicitly
uv run pylint ./rosys                    # not part of pre-commit — run it explicitly
uv run pytest
cd tests && uv run ./test_examples.py    # CI also smoke-runs the examples
```

CI runs these across Python 3.11, 3.12 and 3.13 — don't rely on syntax or deps available in only one of them.

## House style

- Single quotes and f-strings (mark the rare performance-driven exception with a `# NOTE:` comment), `int | None` over `Optional[int]`, 120-column lines.
- `@dataclass(slots=True, kw_only=True)` for value types; match the surrounding file's (sparse) docstring and comment density — prefer clear names over comments.
- In Markdown and docs, write one sentence per line; keep examples in `docs/examples` minimal and focused on a single concept.

## What to avoid

- Blocking the event loop, or `time.sleep` / `asyncio.sleep` in robot code.
- Overwriting persisted state by configuring through `app.on_startup` instead of `rosys.on_startup`.
- Breaking the Hardware/Simulation symmetry (a feature that only works on real hardware).
- Creating new files when editing an existing one suffices; adding unnecessary dependencies.

## Pull requests

Use the repository's [PULL_REQUEST_TEMPLATE.md](.github/PULL_REQUEST_TEMPLATE.md), describe the motivation and implementation, include tests for new features, and make sure the gates above pass.
