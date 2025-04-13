#!/bin/bash
set -e

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
                                --ros-args -p stamped:=true
                                --ros-args -p frame_id:=base_link
