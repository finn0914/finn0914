#!/usr/bin/python3

"""
Simple script for generating a trajectory.
"""


def wrap_angle(theta, min, max):
    while theta > max:
        theta -= 360
    while theta < min:
        theta += 360
    return theta

class TrajectoryBuilder:
    def __init__(self, timestep_ms=100):
        # Settings.
        self.timestep_ms = timestep_ms # [ms]
        # Current trajectory build state.
        self.t_ms = 0                  # [ms]
        self.setpoints = []            # t, x, y, z, yaw [ms, m, m, m, deg]

    def add_first_waypoint(self, x, y, z, yaw):
        self.setpoints.append((self.t_ms, x, y, z, yaw))

    def add_next_waypoint(self, T, x1, y1, z1, yaw1):
        t0, x0, y0, z0, yaw0 = self.setpoints[-1]

        # Generate timepoints.
        timepoints = []
        t1 = t0 + T
        while self.t_ms < t1:
            self.t_ms += self.timestep_ms
            timepoints.append(self.t_ms)

        delta_x, delta_y, delta_z, delta_yaw = x1-x0, y1-y0, z1-z0, yaw1-yaw0
        # Find turn direction that result in smaller angle.
        delta_yaw = wrap_angle(delta_yaw, -180, 180)
        # Linear interpolation.
        L = len(timepoints)
        for i in range(L):
            a = (i+1) / L
            self.setpoints.append((
                timepoints[i],
                x0 + delta_x * a,
                y0 + delta_y * a,
                z0 + delta_z * a,
                wrap_angle(yaw0 + delta_yaw * a, 0, 360)
            ))

    def render_csv(self, filename):
        print("Frequency:             %.1f Hz."%(1000 / self.timestep_ms))
        print("Total trajectory time: %.1f seconds."%(self.setpoints[-1][0] / 1000))
        f = open(filename, "w")
        for p_ms in self.setpoints:
            p = (p_ms[0] / 1000, p_ms[1], p_ms[2], p_ms[3], p_ms[4])
            f.write("%.2f, %.2f, %.2f, %.2f, %.1f\n"%p)
        f.close()

# Edit trajectory sampling frequency, points, and filename here.
def main():
    traj = TrajectoryBuilder(timestep_ms=100)

    traj.add_first_waypoint(      0,  0, 1,   0)
    traj.add_next_waypoint(1000,  0,  0, 1,  90)

    traj.add_next_waypoint(1000,  0, 10, 1,  90)
    traj.add_next_waypoint(1000,  0, 10, 1,   0)

    traj.add_next_waypoint(1000, 10, 10, 1,   0)
    traj.add_next_waypoint(1000, 10, 10, 1, -90)

    traj.add_next_waypoint(1000, 10,  0, 1, -90)
    traj.add_next_waypoint(1000, 10,  0, 1, 180)

    traj.add_next_waypoint(1000,  0,  0, 1, 180)
    traj.add_next_waypoint(1000,  0,  0, 1,  90)
    traj.add_next_waypoint(1000,  0,  0, 1,  90)

    folder = "/home/uav/uav_universe/src/ground-station/station_flight/scripts/trajectory/out/"
    traj.render_csv(folder + "traj1.csv")

if __name__ == "__main__":
    main()
