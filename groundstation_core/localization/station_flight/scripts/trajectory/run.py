import time


def load_trajectory(filename):
    folder = "/home/uav/uav_universe/src/ground-station/station_flight/scripts/trajectory/out/"
    trajectory = []
    try:
        f = open("%s%s"%(folder, filename), "r")
        for line in f:
            tokens = [t.strip() for t in line.split(",")]
            t = float(tokens[0])
            x = float(tokens[1])
            y = float(tokens[2])
            z = float(tokens[3])
            yaw_degree = float(tokens[4])
            trajectory.append((t, x, y, z, yaw_degree))
        f.close()
    except Exception as e:
        print(e)
        return None
    return trajectory

def play_trajectory(stop_event, running_event, trajectory, callback):
    running_event.set()
    try:
        T0 = time.time()
        for t, x, y, z, yaw_degree in trajectory:
            if stop_event.is_set():
                break
            while True:
                T = (time.time() - T0)
                t_remaining = t - T
                if t_remaining <= 0:
                    break
            callback(x, y, z, yaw_degree)
    except Exception as e:
        print(e)
    running_event.clear()
