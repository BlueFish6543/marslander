#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>

using namespace std;

int main() {

  auto start = chrono::system_clock::now();

  // declare variables
  double m, k, x, v, t_max, dt, t, a;
  vector<double> t_list, x_list, v_list;

  // mass, spring constant, initial position and velocity
  m = 1;
  k = 1;
  x = 0;
  v = 1;

  // simulation time and timestep
  t_max = 100;
  dt = 0.1;

//  // Euler integration
//  for (t = 0; t <= t_max; t += dt) {
//
//    // append current state to trajectories
//    t_list.push_back(t);
//    x_list.push_back(x);
//    v_list.push_back(v);
//
//    // calculate new position and velocity
//    a = -k * x / m;
//    x = x + dt * v;
//    v = v + dt * a;
//
//  }

  // Verlet integration
  bool first = true;
  x_list.push_back(x);
  v_list.push_back(v);

  for (t = 0; t <= t_max; t += dt) {
    t_list.push_back(t);

    // for the first time-step, we calculate x using Euler integration
    if (first) {
      x_list.push_back(x + dt * v);
      first = false;
      continue;
    }

    unsigned long size = x_list.size();
    x = 2 * x_list.at(size - 1) - x_list.at(size - 2) +
            pow(dt, 2) * (-k * x_list.at(size - 1)) / m;
    x_list.push_back(x);
    v = (x_list.at(size) - x_list.at(size - 2)) / (2 * dt); // note this is v for the previous time-step
    v_list.push_back(v);
  }

  // need to append v for final time-step
  unsigned long size = x_list.size();
  v = (x_list.at(size - 1) - x_list.at(size - 2)) / dt;
  v_list.push_back(v);

  // Write the trajectories to file
  ofstream fout;
  fout.open("trajectories.txt");
  if (fout) { // file opened successfully
    for (int i = 0; i < t_list.size(); i = i + 1) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  } else { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }

  auto end = chrono::system_clock::now();
  chrono::duration<double> diff = end - start;
  cout << "Time elapsed: " << diff.count() << " seconds" << endl;

  /* The file can be loaded and visualised in Python as follows:

  import numpy as np
  import matplotlib.pyplot as plt
  results = np.loadtxt('trajectories.txt')
  plt.figure(1)
  plt.clf()
  plt.xlabel('time (s)')
  plt.grid()
  plt.plot(results[:, 0], results[:, 1], label='x (m)')
  plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
  plt.legend()
  plt.show()

  */
}
