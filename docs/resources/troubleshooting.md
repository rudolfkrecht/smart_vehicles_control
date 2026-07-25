# Troubleshooting

## Python command not found

On Windows, try `py` instead of `python`. On Ubuntu, try `python3`.

## Virtual environment cannot be activated in PowerShell

Open PowerShell as your normal user and run:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

Then activate the environment again.

## A package is missing

Confirm that the virtual environment is active, then run:

```bash
pip install -r requirements.txt
```

## The simulation window does not appear

First run the non-animated or plotting-only version supplied with the exercise. This allows the controller task to continue while graphics configuration is checked.

