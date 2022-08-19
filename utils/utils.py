import numpy as np

def load_ttl(ttl_path: str):
    return np.loadtxt(ttl_path, dtype=float, delimiter=',', skiprows=1)