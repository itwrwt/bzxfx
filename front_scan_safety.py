#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Small, ROS-independent helpers for the motor bridge's front scan guard."""
from __future__ import annotations

import math


def front_clearance(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    half_angle: float,
) -> tuple[bool, float | None]:
    """Return whether the forward sector has usable beams and its nearest hit."""
    usable = False
    nearest = math.inf
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if abs(math.atan2(math.sin(angle), math.cos(angle))) > half_angle:
            continue
        if math.isinf(distance) and distance > 0:
            usable = True
            continue
        if not math.isfinite(distance) or distance <= 0 or distance > range_max:
            continue
        usable = True
        # A return inside range_min is treated conservatively as very close.
        nearest = min(nearest, max(float(distance), float(range_min)))
    return usable, None if math.isinf(nearest) else nearest
