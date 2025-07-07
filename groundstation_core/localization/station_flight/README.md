# station_flight

Run station with `ros2 run station_flight station.py` and generate trajectory with `ros2 run station_flight generate.py`.

## Code

- `station.py` Ground station.
  - This is where commands are defined.
  - For a brief instruction, see section "Commands" below.
- `my_tcp` (copied from `drone_core`) Handles TCP sockets.
- `config.py` Configurations.
- `image_tcp_receiver.py` Receives image data transferred over TCP.
- `merge_map.py` Create point cloud by merging RGB and depth images.
- `trajectory/` Trajectory generation and loading.
- `util/` (copied from `drone_core`) Utility functions.

### Commands

First, run station. Then, on drones run `flight.py` from `drone_core`, practically using `roslaunch drone_setting run.launch`.

On the station you will be prompted to input commands. For example:
```
>>> r 3        # Reboot drone 3.
>>> o 3        # On drone 3 set origin.
>>> m 3 LOITER # On drone 3 set mode to LOITER.
```

You might find drone grouping useful.
```
>>> ga 1       # Add drone 1 to group.
>>> ga 2       # Add drone 2 to group.
>>> m GUIDED g # Set mode to GUIDED for all drones in group.
```

### Trajectory

Edit `trajectory/generate.py` according to your desired position and yaw, and run it to generate a trajectory CSV file e.g. `traj1.csv`. Then within the station use commands `tl`, `tr`, and `ts` to load, run, and stop trajectory tracking.
