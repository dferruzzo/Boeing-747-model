import numpy as np

from boeing747 import PidDiscreteStandardFiltered


def test_filtered_pid_matches_difference_equation() -> None:
    pid = PidDiscreteStandardFiltered(Kp=1.0, Ki=0.1, Kd=0.01, N=10, Ts=0.1)
    errors = np.array([1.0, 0.2, -0.1])
    outputs = np.array([0.4, -0.3])

    expected = (
        -(pid.a1 / pid.a0) * outputs[0]
        - (pid.a2 / pid.a0) * outputs[1]
        + (pid.b0 / pid.a0) * errors[0]
        + (pid.b1 / pid.a0) * errors[1]
        + (pid.b2 / pid.a0) * errors[2]
    )

    assert pid(errors, outputs) == expected
