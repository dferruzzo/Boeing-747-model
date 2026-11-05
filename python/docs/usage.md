# Usage

Install the package in editable mode:

```bash
python -m pip install -e ".[dev]"
```

Import the dynamics classes from the package:

```python
from boeing747 import Boeing747RotationalDynamics

dynamics = Boeing747RotationalDynamics(Ix=0.247e8, Iy=0.449e8, Iz=0.673e8, Ixz=-0.212e7)
angular_acceleration = dynamics(0.0, [0.0, 0.0, 0.0], [1000.0, 2000.0, 1500.0])
```

Run the simulations from the repository's `python/` directory:

```bash
python examples/simulate_dynamics.py
python examples/pid_response.py
```
