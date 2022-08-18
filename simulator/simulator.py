from math import sqrt
from model import Trajectory, TrajectoryPoint, Vehicle
from dataclasses import dataclass
import numpy as np

@dataclass
class SimulationResult:
    trajectory: Trajectory
    run_time: float
    total_time: float
    average_speed: float
    max_speed: float
    min_speed: float
    max_lat_acc: float
    max_lon_acc: float
    max_lon_dcc: float


class Simulator:
    def __init__(self, inside_bound: Trajectory, outside_bound: Trajectory, vehicle: Vehicle) -> None:
        self.inside_bound = inside_bound
        self.outside_bound = outside_bound
        self.vehicle = vehicle

    def set(trajectory:Trajectory, idx:int, field:int, val: float):
        trajectory.points[idx,field] = val
    
    def get(trajectory:Trajectory, idx:int, field:int):
        return trajectory.points[idx,field]

    def calc_lat_acc(v:float, r:float):
        return v ** 2 / r


    def run_simulation(self, trajectory: Trajectory) -> SimulationResult:
        trajectory_out = Trajectory(len(trajectory.points))
        trajectory_out.points = np.copy(trajectory.points)

        current_radius = Simulator.get(trajectory_out, 0, TrajectoryPoint.CURVATURE)
        current_speed = min(self.vehicle.max_speed_mps, self.vehicle.lookup_speed_from_steer_radius(current_radius))
        current_acc = 0.0
        current_time = 0.0
        trajectory_out.set(0, TrajectoryPoint.SPEED, current_speed)
        trajectory_out.set(0, TrajectoryPoint.LON_ACC, current_acc)
        trajectory_out.set(0, TrajectoryPoint.LAT_ACC, Simulator.calc_lat_acc(current_speed, current_radius))

        num_iteration = 1

        while num_iteration <= 5:
            for i in range(1, len(trajectory_out.points)):
                last_i = i-1
                if i == 0:
                    last_i = -1
                # Find max possible speed
                current_radius = Simulator.get(trajectory_out, 0, TrajectoryPoint.CURVATURE)
                max_speed_curve = self.vehicle.lookup_speed_from_steer_radius(current_radius)

                max_acc = self.vehicle.lookup_acc_from_speed(current_speed)
                displacement = np.linalg.norm(trajectory.points[i, 0:2] - trajectory.points[last_i, 0:2])
                max_speed_acc = sqrt(current_speed ** 2 + 2 * max_acc * displacement)

                if max_speed_acc <= max_speed_curve:
                    # Continue accelerating towards curve's speed limit
                    current_speed = max_speed_acc
                    current_acc = max_acc
                    Simulator.set(trajectory_out, i, TrajectoryPoint.SPEED, current_speed)
                    Simulator.set(trajectory_out, i, TrajectoryPoint.LON_ACC, current_acc)
                    Simulator.set(trajectory_out, i, TrajectoryPoint.LAT_ACC, Simulator.calc_lat_acc(current_speed, current_radius))
                else:
                    max_dcc = self.vehicle.lookup_dcc_from_speed(current_speed)
                    min_speed_dcc = sqrt(current_speed ** 2 + 2 * max_dcc * displacement)
                    if min_speed_dcc <= max_speed_curve:
                        # Follow curve speed
                        current_acc = (max_speed_curve ** 2 - current_speed ** 2) / (2 * displacement)
                        current_speed = max_speed_curve
                        Simulator.set(trajectory_out, i, TrajectoryPoint.SPEED, current_speed)
                        Simulator.set(trajectory_out, i, TrajectoryPoint.LON_ACC, current_acc)
                        Simulator.set(trajectory_out, i, TrajectoryPoint.LAT_ACC, Simulator.calc_lat_acc(current_speed, current_radius))
                    else:
                        # Coming in too fast. Rewind to reduce speed
                        while i > 0:
                            i -= 1
                            
            num_iteration += 1

