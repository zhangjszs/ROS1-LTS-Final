# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS Noetic `catkin` workspace. Main code lives in `src/`.
- `*_core/`: algorithmic modules without ROS dependencies (for example `perception_core`, `planning_core`, `control_core`, `localization_core`).
- `*_ros/`: ROS wrappers (nodes, launch integration, topic/parameter plumbing).
- `src/autodrive_msgs/`: shared message definitions.
- `src/fsd_launch/`: mission/subsystem launch composition and runtime config.
- `docs/`: design and process docs.
- `scripts/` and `perf_reports/`: validation and performance tooling.

Do not edit generated workspace artifacts in `build/`, `devel/`, or `logs/`.

## Build, Test, and Development Commands
Use ROS + catkin tools:

```bash
source /opt/ros/noetic/setup.bash
catkin build --no-status --summarize
source devel/setup.bash
catkin run_tests --no-status --summarize
catkin_test_results build
```

Useful variants:
- `catkin build <package_name>`: build one package.
- `catkin run_tests <package_name>`: run one package’s tests.
- `roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to.bag`: local mission replay.

## Coding Style & Naming Conventions
Target standard is C++17. Formatting is defined in `.clang-format` (Google-based): 2-space indent, 100-column limit, attached braces, left-aligned pointers/references.
- Run `clang-format -i <file>` before committing.
- Prefer lower_snake_case for files/functions/variables.
- Use UpperCamelCase for types/classes.
- Keep package and ROS namespace names descriptive and stable.

## Testing Guidelines
Tests are package-local, usually under `src/<pkg>/test/`, and primarily use GTest via catkin.
- Name tests `test_<feature>.cpp` (for example `test_line_detection.cpp`).
- Add/extend unit tests in `*_core/test` when changing algorithms.
- Add wrapper/integration tests in `*_ros/test` when changing ROS interfaces.
- Treat planner/localization parameter changes as regression-sensitive; include replay validation when possible.

## Commit & Pull Request Guidelines
Recent history follows Conventional Commit style:
- `feat(planning): ...`
- `fix(localization): ...`
- `chore(config): ...`
- `docs: ...`

Keep commits scoped to one logical change. PRs should include:
- concise problem/solution summary,
- impacted packages and launch/config files,
- exact validation commands run (and results),
- linked issue/task, plus screenshots when visualization behavior changes.
