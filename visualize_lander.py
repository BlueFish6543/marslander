import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
results = np.absolute(np.loadtxt('autopilot.txt'))
nrows = results.shape[0]
indices = np.expand_dims(np.arange(nrows), 1)
results = np.concatenate((indices, results), axis=1)
plt.figure(1)
plt.clf()
plt.xlabel('Time step')
plt.ylabel('Descent speed (m/s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='Actual')
plt.plot(results[:, 0], results[:, 2], label='Target')
plt.legend()
plt.show()
