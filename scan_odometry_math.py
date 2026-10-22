#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure helpers for finite-difference laser odometry."""
from __future__ import annotations

import math


def wrapped_angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))
