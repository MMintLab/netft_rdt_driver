import rospy
import time
import numpy as np
import copy
from typing import Optional

from threading import Lock
from collections import deque

from netft_rdt_driver.srv import Zero
from geometry_msgs.msg import WrenchStamped


class FTSensor(object):
    def __init__(self, ns='netft', wait_for_data=False, buffer_size=50):
        self.ns = ns
        self.topic_name = '/{}/netft_data'.format(self.ns)
        self.buffer_size = buffer_size
        self._data = None
        self.lock = Lock()
        self.clean_buffer() # reset buffer
        self.ft_data_subscriber = self._get_ft_data_subscriber()
        self.get_wrench(block_until_data=wait_for_data)

    @property
    def data(self):
        with self.lock:
            return copy.deepcopy(self._data)

    def clean_buffer(self):
        with self.lock:
            self._data = deque([None] * self.buffer_size, self.buffer_size)  # clean buffer

    def _get_ft_data_subscriber(self):
        ft_data_subscriber = rospy.Subscriber(self.topic_name, WrenchStamped, self._ft_data_callback)
        return ft_data_subscriber

    def _ft_data_callback(self, msg):
        with self.lock:
            msg.header.frame_id = '{}_sensor'.format(self.ns)
            self._data.append(msg)

    def zero(self):
        """
        Zero the sensor -- call the service
        :return: None
        """
        service_name = '/{}/zero'.format(self.ns)
        rospy.wait_for_service(service_name)
        try:
            zero_proxy = rospy.ServiceProxy(service_name, Zero)
            zero_proxy()
            self.clean_buffer()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_wrench(self, average=False, block_until_data=True):
        """
        Get the wrench
        Args:
            average:
            block_until_data:

        Returns:

        """
        if average:
            wait_for(lambda: not(block_until_data and None in self.data), 10, f"FTSensor({self.ns})")
            return average_wrench_stamped(self.data)
        else:
            wait_for(lambda: not(block_until_data and self.data[-1] is None), 10, f"FTSensor({self.ns})")
            return self.data[-1] # get the most recent wrench stamped


# USEFUL FUNCTIONS:

def average_wrench_stamped(wrench_stampeds):
    # wrench_stampeds: list of WrenchStamped
    avg_wrench_stamped = WrenchStamped()
    all_data = np.asarray([[ws.wrench.force.x,
                            ws.wrench.force.y,
                            ws.wrench.force.z,
                            ws.wrench.torque.x,
                            ws.wrench.torque.y,
                            ws.wrench.torque.z] for ws in wrench_stampeds]) # fx fy fz tx tx tz
    data_avg = np.mean(all_data, axis=0)
    # pack data:
    avg_wrench_stamped.header = wrench_stampeds[-1].header # use the header info from the last wrench (causal filter)
    avg_wrench_stamped.wrench.force.x = data_avg[0]
    avg_wrench_stamped.wrench.force.y = data_avg[1]
    avg_wrench_stamped.wrench.force.z = data_avg[2]
    avg_wrench_stamped.wrench.torque.x = data_avg[3]
    avg_wrench_stamped.wrench.torque.y = data_avg[4]
    avg_wrench_stamped.wrench.torque.z = data_avg[5]
    return avg_wrench_stamped


# From arc_utilities
def wait_for(func, warn_after: Optional[int] = 10, name: Optional[str] = ""):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.

    Introduces sleep delay, not recommended for time critical operations
    """
    start_t = rospy.Time.now()
    while not func() and not rospy.is_shutdown():
        if warn_after is not None and rospy.Time.now() - start_t > rospy.Duration(secs=warn_after):
            warning = f"still waiting after {warn_after}s"
            if name:
                warning += f" for {name}"
            rospy.logwarn_throttle(5, warning)
        time.sleep(0.01)


# DEBUG:
if __name__ == '__main__':
    rospy.init_node('ft_sensor')
    ftsensor = FTSensor(ns='netft')
    avg_wrench_stamped = ftsensor.get_wrench(average=True)
