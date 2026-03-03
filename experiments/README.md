# Experiments

This is the Mac-only project for ad hoc experiments.

Use this directory for dependencies that should not be added to the Pi runtime in `../pi/`.

Examples:

```bash
cd experiments
uv add <package>
uv sync
```

Rule of thumb:

- add to `pi/` only if the Pi needs it
- add to `experiments/` for local Mac-only work
