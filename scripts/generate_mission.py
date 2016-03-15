#!/usr/bin/env python
import json
import numpy as np

np.random.seed(42)

n = 50
ips = np.zeros((n, 6))

ips[:, 0] = np.random.randint(-50, 50, n)
ips[:, 1] = np.random.randint(-50, 50, n)
ips[:, 2] = 3.0
ips[:, 5] = np.deg2rad(np.random.randint(-180, 180, n))

mission = {
    'ips': ips.tolist()
}

print(json.dumps(mission, indent=2))



