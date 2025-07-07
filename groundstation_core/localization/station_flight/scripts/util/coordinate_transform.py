"""
Copied from repo `drone_core main c0fa0af`.
"""

import math
import numpy as np


def enu_to_ned(e, n, u):
    return (n, e, -u)

def ned_to_enu(n, e, d):
    return (e, n, -d)

# See https://math.stackexchange.com/a/949185
# Assuming roll and pitch in [-pi, pi].
def roll_pitch_to_tilt(roll_radian, pitch_radian):
    return math.acos(math.cos(roll_radian) * math.cos(pitch_radian))
