# Codebase Coding Standards

Apply this to custom packages only.

## Package Standard

- Prefer `include/<package>/`, `src/`, `config/`, `launch/`, and `README.md`.
- Public headers live in `include/<package>/...`.
- Source files include headers as `"package/header.hpp"`.
- Launch-only packages should not keep empty `include/` or `src/` directories.
- If a runtime package exposes parameters, it should have a YAML config in `config/`.

## Header / Source Standard

- `.hpp` files contain declarations, public types, and member variables only.
- `.cpp` files contain implementations only.
- One main node/class should have one main header and one main source file.
- Do not keep declarations for methods that are not defined and used.
- Remove unused members, locals, helper functions, and includes.

## Helper Function Standard

- Use an anonymous namespace for file-local helpers.
- Use private class methods when helper logic needs class state.
- Use `*_helpers.cpp` for substantial shared math, conversion, scoring, or reconstruction logic.
- Use `*_helpers.hpp` only for shared types or declarations needed by more than one `.cpp`.
- Keep ROS node orchestration in the main `.cpp`.

## ROS Interface Standard

- Topic names, service names, action names, and frame names should be declared as parameters with code defaults.
- Tunable runtime values should also be parameters.
- Internal constants that are not meant to change can stay as local `constexpr` values in `.cpp`.
- Parameter overrides should normally live in package YAML config files.
- Launch files should load config YAML and only override parameters directly when launch-specific behavior really differs.

## File Responsibility Standard

- Main node `.cpp`: constructor, subscriptions, publishers, timers, action/service handlers, top-level control/planning flow.
- Helper `.cpp`: reusable internal math, conversions, scoring, and reconstruction.
- Config YAML: runtime interface names and tunables.
- Launch files: compose nodes, load config, expose high-level launch arguments only.
- README: package purpose, how to run, default interfaces, config path, and one known-good example.

## README Standard

- Keep it short.
- Include:
  - package purpose
  - main executable or launch file
  - config file path
  - default topics/services/actions
  - one example command

## Naming Standard

- Parameter names: lowercase snake_case
- Topic/service/action parameters: `*_topic`, `*_service`, `action_name`
- One naming style per package for nodes, topics, and helper files
- Prefer package-scoped include paths everywhere
