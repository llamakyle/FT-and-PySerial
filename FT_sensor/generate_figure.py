import matplotlib.pyplot as plt
import scipy.io
import os
curr_dir = os.path.dirname(os.path.realpath(__file__))

# Connect to existing docker:
# https://stackoverflow.com/a/30173220

print("Read FT sensor readings ...")
FT_readings = scipy.io.loadmat(curr_dir + '/FT_results/FT_readings.mat')

# Plot FT sensor readings
arr_ylabel = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
for iterr in range(6):
    ax = plt.subplot(2, 3, iterr+1)
    ax.plot(FT_readings['FT_time_seq'][0], FT_readings['FT_meas_log'][:, iterr])
    ax.set_xlabel('Time/sec')
    ax.set_ylabel(arr_ylabel[iterr])
    ax.grid()

fig = plt.gcf()
fig.set_size_inches((20, 8.5), forward=False)

print("Saving a figure for FT sensor readings ...")
fig.savefig(curr_dir + '/FT_results/FT_readings.png', dpi=500)

plt.show()
