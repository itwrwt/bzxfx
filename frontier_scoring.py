#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure scoring helper for selecting less turn-heavy exploration goals."""
from __future__ import annotations


def frontier_score(
    cluster_size: int,
    distance: float,
    heading_error: float,
    turn_cost_m_per_rad: float = 0.8,
) -> float:
    """Balance frontier information against travel distance and required turn."""
    cost = distance + 0.5 + turn_cost_m_per_rad * abs(heading_error)
    return float(cluster_size) / max(cost, 1e-6)
