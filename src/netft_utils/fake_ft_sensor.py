import rospy
import time
import numpy as np
import copy
from typing import Optional

from threading import Lock
from collections import deque

from netft_rdt_driver.srv import Zero
from geometry_msgs.msg import WrenchStamped

from netft_utils.ft_sensor import FTSensor


class FakeFTSensor(FTSensor):

    def _get_ft_data_subscriber(self):
        fake_ft_data_subscriber = None
        return fake_ft_data_subscriber

    def clean_buffer(self):
        with self.lock:
            self._data = deque([get_fake_wrench()] * self.buffer_size, self.buffer_size)  # clean buffer

    def zero(self):
        """
        Zero the sensor -- call the service
        :return: None
        """
        # Fake zeroing the sensor
        pass


# USEFUL FUNCTIONS:
def get_fake_wrench():
    # wrench_stampeds: list of WrenchStamped
    fake_wrench_stamped = WrenchStamped()
    return fake_wrench_stamped

