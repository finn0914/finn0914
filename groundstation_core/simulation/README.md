# ground-station/simulation

## gym-pybullet-drones

The current code base depends on an unknown version of [`gym-pybullet-drones`](https://github.com/utiasDSL/gym-pybullet-drones/tree/main), and its relevant content is vendored. Compared to `main`, There are discrepancies in e.g. the description of required library version, but it runs fine for now. TODO

## venv

You may want to run in a virtual environment e.g. due to version incompatibility. Read e.g. https://python.land/virtual-environments/virtualenv on how to use `venv`.

```
cd simulation
python3.10 -m venv venv
source venv/bin/activate # Later use `deactivate` to exit virtual environment.

# pybullet: "numpy<2.0"
# open3d: python<=3.11
# gym_pybullet_drones: python<=3.10 (not sure)
python3 -m pip install "numpy<2.0" pybullet matplotlib open3d gymnasium transforms3d gym opencv-python casadi do_mpc shapely PyQt6
python3 fly_clutter_replan.py
```

### Ubuntu

To install older python versions, you may consider using `deadsnakes`, a well-maintained Python repo for Ubuntu, cf. https://www.geeksforgeeks.org/how-to-install-python-on-linux/

## Run

1. Edit `config.py`.
2. Run `python3 fly_clutter_replan.py`
3. Check **addUserDebugPoints** if errors.

## Notice

There are two ways to construct the map, one is from the sensed environment in the simulator and convert it to the map by **map_2d = obstacle_processor.update_point_cloud_observation(j, pcd,agentstate[:3],agentstate[9],voxel_size)**; another is using the **known_map_establish** to create the environment, and then update the environment to the **user_map** (which is used when planning) by **update_user_map**.

_Usually we use the first method._
