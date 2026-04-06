## Experiments

This directory stores **experiment indices** only (inputs, command templates, and output naming conventions).
Large local assets are intentionally excluded from git:

- Inputs: `bags/`, `maps/`
- Outputs: `eval/`

Each experiment folder should include:

- `inputs.md`: where the input bag/pbstream comes from and required topics/frames.
- `commands.md`: the exact commands to reproduce the runs.
- `outputs.md`: expected output bag/TUM filenames and evaluation notes.

