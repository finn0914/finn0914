#!/usr/bin/env bash
#
# kill_sitl_ros2.sh
# Brutally or politely stop everything related to:
#   – PX4 SITL
#   – Gazebo (classic & Ignition/“gz”)
#   – ROS 2 nodes, tools, and daemon

set -euo pipefail

echo "🛑  Stopping PX4 SITL, Gazebo & ROS 2…"

# ------------------------------------------------------------------------------
# Helper: stop the ROS 2 daemon so that *all* nodes lose discovery and exit
# ------------------------------------------------------------------------------
stop_ros2_daemon() {
  if pgrep -f 'ros2 daemon' >/dev/null 2>&1; then
    echo "→ Stopping ROS 2 daemon (ros2 daemon stop)…"
    ros2 daemon stop || true
    # Give it a moment to exit
    sleep 2
  fi
}

# ------------------------------------------------------------------------------
# Process-name patterns to kill (regexes for pgrep/pkill -f)
# ------------------------------------------------------------------------------
patterns=(
  '^px4$' '^px4_'                   # PX4 SITL and per-instance wrapper
  '^gazebo$' '^gz$' '^gzserver$' '^gzclient$' '^ign[- ]gazebo'
  '^ros2' 'roslaunch' 'roscore' 'rosbag' '^rviz2$' '^rqt'
  'dds.*Discovery'  'fastdds'       # common DDS daemons
)

# ------------------------------------------------------------------------------
# First politely ask the daemon to stop (if any)
# ------------------------------------------------------------------------------
stop_ros2_daemon

# ------------------------------------------------------------------------------
# Then brute-force kill the matching processes
# ------------------------------------------------------------------------------
for pat in "${patterns[@]}"; do
  printf "→ Killing processes matching %-25s " "\"$pat\""
  if pkill -9 -f "$pat" 2>/dev/null; then
    echo "killed"
  else
    echo "none"
  fi
done

# ------------------------------------------------------------------------------
# One more try: stop daemon again in case it re-spawned
# ------------------------------------------------------------------------------
stop_ros2_daemon

# ------------------------------------------------------------------------------
# Verification
# ------------------------------------------------------------------------------
echo; echo "🔍 Verifying that nothing remains…"
if pgrep -fl 'px4|gz|gazebo|ign|ros2|roslaunch|roscore|rviz2|rqt|rosbag|fastdds|dds.*Discovery' >/dev/null; then
  echo "⚠️  Some matching processes are still running."
  exit 1
else
  echo "✅  All PX4, Gazebo, and ROS 2 processes have terminated."
  exit 0
fi
