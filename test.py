from simulator.model.trajectory import Trajectory
from simulator.model.vehicle import Vehicle
from utils.utils import load_ttl
from simulator.simulator import SimulationResult, Simulator
import numpy as np
import sys

sys.path.append("/home/haoru/Projects/IAC/offline-trajectory-optimization")

ttl = load_ttl("purdue_ttl.csv")
traj = Trajectory(len(ttl))
traj[:, 0:2] = ttl[:, 0:2]

vehicle = Vehicle(
    downforce_speed_lookup=np.array([[], []]),
    steer_radius_speed_lookup=np.array([[], []]),
    acc_speed_lookup=np.array([[0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0], [
                              15.0, 12.0, 9.0, 6.0, 3.0, 0.0, 0.0]]),
    dcc_speed_lookup=np.array([[0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0],
                              [-15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0]]),
    g_circle_radius_mpss=20.0
)

simulator = Simulator(vehicle)
result = simulator.run_simulation(traj, enable_vis=True)
print(result)
