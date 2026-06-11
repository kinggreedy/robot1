# CircuitPython editor setup

This workspace is configured for CircuitPython analysis with VS Code.

## What was added

- `.vscode/settings.json` points VS Code to the local `.venv` interpreter.
- `python.analysis.extraPaths` includes the workspace `lib/` folder so bundled CircuitPython modules can be resolved.

## If VS Code still shows unresolved imports

1. Open the command palette.
2. Run `Python: Select Interpreter`.
3. Choose `c:\Data\bot\simple_bot\.venv\Scripts\python.exe`.
4. Reload the window.
5. If needed, install CircuitPython stubs into the selected environment for better type information.

## Optional stub install

If you want stronger analysis, install CircuitPython stubs into the venv, for example:

```powershell
c:\Data\bot\simple_bot\.venv\Scripts\python.exe -m pip install circuitpython-stubs
```

If that package is unavailable in your environment, you can still rely on the `lib/` folder plus the board running code on the Pico.
