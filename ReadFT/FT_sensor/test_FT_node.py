# This script is for testing. It is not executed within the main process

import multiprocessing
import FT_ros_interface as ros_interface
import launch_FT_sensor as launch_ft_sensor
from events import event_shutdown_FT
import runpy
import time

# manager = multiprocessing.Manager()
# return_dict = manager.dict()

p_launch = multiprocessing.Process(target=launch_ft_sensor.launch_ft_node, args=(event_shutdown_FT,))
p_launch.start()

time.sleep(10)  # Wait until the FT node is fully launched

p_FT = multiprocessing.Process(target=ros_interface.measure_force, args=(event_shutdown_FT,))
p_FT.start()

for iterr in range(30):
    # print("=========================================================================================================")
    time.sleep(1)

event_shutdown_FT.set()
time.sleep(8)
print("Shut down FT launch node!!!")
p_FT.terminate()
p_launch.terminate()

runpy.run_module('generate_figure')
