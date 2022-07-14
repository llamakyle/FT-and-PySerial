import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
import numpy as np



#SG Setup

# Settings for plotting
plot_window = 200
y_var = np.array(np.zeros([plot_window]))
y_var2 = np.array(np.zeros([plot_window]))

plt.ion()
fig, ax = plt.subplots()

line, = ax.plot(y_var,'r')
line2, = ax.plot(y_var2,'g')


while True:
	y_var = np.append(y_var,var1)
	y_var2 = np.append(y_var2,var2)
	y_var = y_var[1:plot_window+1]
	y_var2 = y_var2[1:plot_window+1]
	line.set_ydata(y_var)
	line2.set_ydata(y_var2)


	ax.relim()
	ax.autoscale_view()
	fig.canvas.draw()
	fig.canvas.flush_events()	
