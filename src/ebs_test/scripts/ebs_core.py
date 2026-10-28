#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-free EBS diagnostics. Never infer hardware events from geometry."""
from __future__ import division
import math

def finite(values):
    return all(not (math.isnan(v) or math.isinf(v)) for v in values)

def require_shadow_only(requested):
    # Compatibility helper for historical shadow/model validation. Actual
    # output policy is validate_output_request in ebs_output.py.
    if requested:
        raise RuntimeError('EBS vehicle output unavailable: RES Go/EBS/steering contract not integrated')

class CourseProgress(object):
    def __init__(self, acceleration=25.0, braking=10.0):
        if not finite((acceleration, braking)) or min(acceleration, braking) <= 0:
            raise ValueError('Course lengths must be positive and finite')
        self.acceleration = acceleration
        self.braking = braking
        self.origin = None
        self.yaw = None
        self.progress = None

    def observe(self, x, y, axis_yaw, fresh=True):
        if not fresh or not finite((x, y, axis_yaw)):
            return False
        if self.origin is None:
            self.origin, self.yaw = (x, y), axis_yaw
        self.progress = ((x-self.origin[0])*math.cos(self.yaw) +
                         (y-self.origin[1])*math.sin(self.yaw))
        return True

    def snapshot(self, fresh=False):
        s = self.progress
        region = 'unknown'
        if fresh and s is not None:
            region = ('before_origin' if s < 0 else
                      'acceleration_region' if s < self.acceleration else
                      'braking_region' if s <= self.acceleration+self.braking else
                      'beyond_course')
        return dict(progress_m=s, progress_fresh=bool(fresh), geometric_region=region,
                    hardware_go=None, hardware_ebs_active=None, hardware_emergency=None,
                    hardware_standstill=None, actual_speed_mps=None,
                    hardware_state_source='not_integrated',
                    region_is_brake_trigger=False, test_passed=None)
