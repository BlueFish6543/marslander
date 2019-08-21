# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 1000
dt = 1
t_array = np.arange(0, t_max, dt)

'''
Analytical solution:
x = x sin (sqrt(k / m) t)
v = v cos (sqrt(k / m) t)
'''


def plot(t_array, x_array, v_array, title):
    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.title(title)
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()


def main(x, v):

    # # initialise empty lists to record trajectories
    # x_list = []
    # v_list = []
    #
    # # Euler integration
    # for t in t_array:
    #
    #     # append current state to trajectories
    #     x_list.append(x)
    #     v_list.append(v)
    #
    #     # calculate new position and velocity
    #     a = -k * x / m
    #     x = x + dt * v
    #     v = v + dt * a
    #
    # # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    # x_array = np.array(x_list)
    # v_array = np.array(v_list)
    #
    # plot(t_array, x_array, v_array, "Euler")
    #
    # x, v = x_list[0], v_list[0]  # obtain original variables x and v

    # Verlet integration
    x_list, v_list = [x], [v]
    first = True

    for t in range(t_array.size - 1):
        # for the first time-step, we calculate x using Euler integration
        if first:
            x_list.append(x + dt * v)
            first = False
            continue

        x = 2 * x_list[-1] - x_list[-2] + dt ** 2 * (-k * x_list[-1]) / m
        x_list.append(x)
        v = (x_list[-1] - x_list[-3]) / (2 * dt)  # note this is v for the previous time-step
        v_list.append(v)

    # need to append v for final time-step
    v = (x_list[-1] - x_list[-2]) / dt
    v_list.append(v)

    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    # plot(t_array, x_array, v_array, "Verlet")

    # note: Verlet is stable for dt = 1.99 but unstable for dt = 2
    # thus critical value is around 2


if __name__ == '__main__':
    start = time.time()
    main(x, v)
    end = time.time()
    print("Time taken: %.5f seconds" % (end - start))
