# Boeing 747 model

Python implementation of Boeing 747 translational, rotational, and Euler-angle
dynamics, with a filtered discrete PID controller.

## Installation

Use the project's virtual environment explicitly:

```bash
.venv/bin/python -m pip install -e ".[dev]"
```

Alternatively, activate it before using `python`:

```bash
source .venv/bin/activate
python -m pip install -e ".[dev]"
```

## Run tests

```bash
.venv/bin/python -m pytest
```

Run a single test file:

```bash
.venv/bin/python -m pytest tests/test_boeing747_dynamics.py
```

## Run examples

```bash
.venv/bin/python examples/simulate_dynamics.py
.venv/bin/python examples/pid_response.py
```

See [docs/usage.md](docs/usage.md) for package imports and
[docs/equations.md](docs/equations.md) for model conventions.
