# Software setup

The exercises use a lightweight Python environment that runs on Windows and Ubuntu. ROS, MATLAB and a large driving simulator are not required.

## Required software

- Python 3.11 or 3.12
- A code editor, preferably Visual Studio Code
- Git

## Download the course

=== "Git"

    ```bash
    git clone https://github.com/rudolfkrecht/smart_vehicles_control.git
    cd smart_vehicles_control
    ```

=== "ZIP download"

    Download the repository from GitHub using **Code → Download ZIP**, extract it and open the extracted folder.

## Create a virtual environment

=== "Windows PowerShell"

    ```powershell
    py -3.12 -m venv .venv
    .\.venv\Scripts\Activate.ps1
    python -m pip install --upgrade pip
    pip install -r requirements.txt
    ```

=== "Ubuntu"

    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip
    pip install -r requirements.txt
    ```

## Verify the installation

```bash
python setup_check.py
```

The expected final line is:

```text
Setup check passed.
```

!!! warning
    Exercise files and `requirements.txt` will be added with the Day 1 practical package. The documentation site itself can already be previewed using the commands below.

## Preview this website locally

```bash
pip install -r requirements-docs.txt
mkdocs serve
```

Open the local address printed in the terminal, normally `http://127.0.0.1:8000`.

