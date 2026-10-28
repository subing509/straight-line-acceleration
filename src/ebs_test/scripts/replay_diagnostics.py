#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""CSV geometric diagnostics only; no ROS import, controller or CAN transport."""
from __future__ import print_function
import argparse
import csv
import json
from ebs_core import CourseProgress

def replay(csv_path):
    course=CourseProgress()
    regions={};count=0;last=None
    with open(csv_path, 'r') as stream:
        for row in csv.DictReader(stream):
            # Freeze the first recorded heading, explicitly not a surveyed axis.
            accepted=course.observe(float(row['vehicle_x']),float(row['vehicle_y']),
                                    float(row['vehicle_yaw']))
            last=course.snapshot(accepted)
            key=last['geometric_region'];regions[key]=regions.get(key,0)+1
            count+=1
    return dict(records=count,regions=regions,last=last,
                axis_source='first_recorded_vehicle_heading_not_surveyed_course',
                replay_scope='geometric_diagnostics_only_not_PD_or_sensor_replay',
                vehicle_commands_sent=0,hardware_states_inferred=False)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('csv');parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=replay(args.csv)
    with open(args.output,'w') as out:json.dump(result,out,indent=2,sort_keys=True)
    print(json.dumps(result,sort_keys=True))
