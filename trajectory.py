from __future__ import print_function
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
# all in SI units
m = 100
M = 6.42e23  # mass of Mars
G = 6.67e-11
x = np.array([0, 5e7, 0])  # initial position
v = np.array([0, 0, 0])  # initial velocity
v_circular = np.array([np.sqrt(G * M / x[1]), 0, 0])  # velocity for circular orbit
v_elliptical = 1.25 * v_circular
v_escape = np.array([np.sqrt(2 * G * M / x[1]), 0, 0])  # escape velocity


def f(x):
    # calculates force vector
    return - (G * M * m / (np.linalg.norm(x) ** 3)) * x


# simulation time, timestep and time
t_max = 1e6
dt = 1
t_array = np.arange(0, t_max, dt)


def euler(x, v):

    position_list = []
    velocity_list = []

    # Euler integration
    for t in t_array:

        # append current state to trajectories
        position_list.append(x)
        velocity_list.append(v)

        # calculate new position and velocity
        a = f(x) / m
        x = x + dt * v
        v = v + dt * a

    return position_list


def verlet(x, v):

    position_list, velocity_list = [x], [v]
    first = True

    for t in range(t_array.size - 1):
        # for the first time-step, we calculate x using Euler integration
        if first:
            position_list.append(x + dt * v)
            first = False
            continue

        x = 2 * position_list[-1] - position_list[-2] + dt ** 2 * f(position_list[-1]) / m
        position_list.append(x)
        v = (position_list[-1] - position_list[-3]) / (2 * dt)  # note this is v for the previous time-step
        velocity_list.append(v)

    # need to append v for final time-step
    v = (position_list[-1] - position_list[-2]) / dt
    velocity_list.append(v)

    return position_list


def main():
    # position_list = euler(x, v)
    position_list = verlet(x, v_escape)

    xs = np.array([position[0] for position in position_list])
    ys = np.array([position[1] for position in position_list])

    plt.scatter(xs, ys)
    plt.axis('scaled')
    plt.show()


if __name__ == '__main__':
    main()
