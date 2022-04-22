import os
import sys
dir_this_folder = os.path.dirname(os.path.realpath(__file__))
import roslaunch
import time


def launch_ft_node(event_shutdown):

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # TODO: I have to go to $ROS_ROOT/config and change the rosconsole.config from INFO to FATAL:
    #  log4j.logger.ros=FATAL (ref: https://github.com/ros/ros_comm/issues/139#issuecomment-12959893)
    #  But this will turn off all warnings. Is there a better way?
    # Also, in the command line, we can add '2>/dev/null' to the roslaunch command to suppress all warnings
    launch = roslaunch.parent.ROSLaunchParent(uuid, [dir_this_folder + "/launch_FT_sensor.launch"])
    launch.start()

    event_shutdown.wait()

    time.sleep(1)
    print("Exiting launch !!")
    launch.shutdown()


if __name__ == "__main__":
    launch_ft_node(1)
