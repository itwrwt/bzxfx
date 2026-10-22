#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Map Twist requests to the chassis' discrete serial motion commands."""
from __future__ import annotations


def should_stop_for_front_obstacle(motion_key: str) -> bool:
    """The front guard stops forward drive, while permitting a pivot turn."""
    return motion_key == "w"


def key_for_twist(
    vx: float,
    wz: float,
    deadband: float = 0.03,
    allow_backward: bool = False,
    forward_blocked: bool = False,
    turn_threshold: float = 0.55,
) -> str:
    """Quantize Nav2 twists for a chassis with discrete direction commands."""
    turning = abs(wz) >= turn_threshold
    if vx < -deadband and allow_backward and not turning:
        return "x"
    if forward_blocked:
        if turning:
            return "l" if wz > 0 else "r"
        return "s"
    if abs(vx) <= deadband:
        if turning:
            return "l" if wz > 0 else "r"
        return "s"
    if vx > deadband:
        if turning:
            return "l" if wz > 0 else "r"
        return "w"
    return "s"
