#!/usr/bin/env python3

from math import sqrt

def euclidean_distance_between_points(p1, p2):
    if len(p1) == 2 and len(p2) == 2:
        if isinstance(p1, dict):
            x1 = p1["x"]
            y1 = p1["y"]
            z1 = 0
        elif isinstance(p1, list) or isinstance(p1, tuple):
            x1 = p1[0]
            y1 = p1[1]
            z1 = 0
        else:
            raise ValueError("Invalid point format")
        if isinstance(p2, dict):
            x2 = p2["x"]
            y2 = p2["y"]
            z2 = 0
        elif isinstance(p2, list) or isinstance(p2, tuple):
            x2 = p2[0]
            y2 = p2[1]
            z2 = 0
        else:
            raise ValueError("Invalid point format")
    elif len(p1) == 3 and len(p2) == 3:
        if isinstance(p1, dict):
            x1 = p1["x"]
            y1 = p1["y"]
            z1 = p1["z"]
        elif isinstance(p1, list) or isinstance(p1, tuple):
            x1 = p1[0]
            y1 = p1[1]
            z1 = p1[2]
        else:
            raise ValueError("Invalid point format")
        if isinstance(p2, dict):
            x2 = p2["x"]
            y2 = p2["y"]
            z2 = p2["z"]
        elif isinstance(p2, list) or isinstance(p2, tuple):
            x2 = p2[0]
            y2 = p2[1]
            z2 = p2[2]
        else:
            raise ValueError("Invalid point format")
    else:
        raise ValueError("Invalid point format")
    return sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

def cost_between_points(p1, p2, penalize_z_movement=1.0):
    if len(p1) == 2 and len(p2) == 2:
        if isinstance(p1, dict):
            x1 = p1["x"]
            y1 = p1["y"]
            z1 = 0
        elif isinstance(p1, list) or isinstance(p1, tuple):
            x1 = p1[0]
            y1 = p1[1]
            z1 = 0
        else:
            raise ValueError("Invalid point format")
        if isinstance(p2, dict):
            x2 = p2["x"]
            y2 = p2["y"]
            z2 = 0
        elif isinstance(p2, list) or isinstance(p2, tuple):
            x2 = p2[0]
            y2 = p2[1]
            z2 = 0
        else:
            raise ValueError("Invalid point format")
    elif len(p1) == 3 and len(p2) == 3:
        if isinstance(p1, dict):
            x1 = p1["x"]
            y1 = p1["y"]
            z1 = p1["z"]
        elif isinstance(p1, list) or isinstance(p1, tuple):
            x1 = p1[0]
            y1 = p1[1]
            z1 = p1[2]
        else:
            raise ValueError("Invalid point format")
        if isinstance(p2, dict):
            x2 = p2["x"]
            y2 = p2["y"]
            z2 = p2["z"]
        elif isinstance(p2, list) or isinstance(p2, tuple):
            x2 = p2[0]
            y2 = p2[1]
            z2 = p2[2]
        else:
            raise ValueError("Invalid point format")
    else:
        raise ValueError("Invalid point format")
    return sqrt((x1 - x2)**2 + (y1 - y2)**2 + (penalize_z_movement*(z1 - z2))**2)