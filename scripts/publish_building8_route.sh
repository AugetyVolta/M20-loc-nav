#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/source_m20_nav.sh"

# Building 8 pose captured from the live RViz interactive marker.
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 58.26122283935547, y: 0.7538578510284424, z: 14.217455863952637}, orientation: {x: 1.190658387216673e-7, y: 1.7270651668914694e-10, z: 0.998107140692798, w: 0.06149907070869339}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}"

ros2 topic pub --once /waypoint_sequence/clear std_msgs/msg/Empty "{}"

ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: -2.9947586059570312, y: -0.16126275062561035, z: 0.1943725526332855}}"
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 63.7553596496582, y: 62.41653060913086, z: 0.1140154480934143}}"
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 52.393333435058594, y: 0.9068405628204346, z: 14.099128532409668}}"

printf 'Published Building 8 initial pose and 3 waypoints.\n'
