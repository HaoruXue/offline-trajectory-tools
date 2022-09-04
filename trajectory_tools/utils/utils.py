import numpy as np
from trajectory_tools.simulator.model import trajectory
from trajectory_tools.simulator.model.trajectory import Trajectory


def load_ttl(ttl_path: str) -> Trajectory:
    with open(ttl_path, "r") as f:
        header = f.readline().split(",")
        assert len(header) >= 3
        data = np.loadtxt(ttl_path, dtype=float, delimiter=",", skiprows=1)
        trajectory = Trajectory(len(data), int(header[0]))
        assert data.shape == trajectory.points.shape
        trajectory.points = data
        return trajectory


def save_ttl(ttl_path: str, trajectory: Trajectory):
    with open(ttl_path, "w") as f:
        f.write(
            ",".join(
                [
                    str(trajectory.ttl_num),
                    str(len(trajectory)),
                    str(trajectory[0, Trajectory.DIST_TO_SF_FWD]),
                ]
            )
        )
        f.write("\n")
        np.savetxt(f, trajectory.points, delimiter=",")
