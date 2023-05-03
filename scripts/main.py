#!/usr/bin/env python
import os
import sys
import yaml
import socket
import threading
from collections import deque

import numpy as np

import rospy
import rospkg

from std_msgs.msg import Header
from sensor_msgs.msg import PointField, PointCloud2

# messing around with paths to get imports to work
try:
    rospack = rospkg.RosPack()
    pckg_path = rospack.get_path('velarray')
    sys.path.append(pckg_path)
    from src.velarray_datapackets import *
except Exception as e:
    raise e

_MTU = 2048
'''maximum transferrable unit'''

PORT = 2368
'''default port used by the velarray'''


class AverageFrameRate:
    def __init__(self, maxlen: 'int | None' = None):
        self._buffer = deque(maxlen=maxlen)
        self._last_timestamp = None
        self._acc = 0

    def new_timestamp(self, __ts: rospy.Time) -> None:

        if self._last_timestamp is not None:
            d = (__ts - self._last_timestamp).to_sec()
            if len(self._buffer) == self._buffer.maxlen:
                self._acc -= self._buffer[0]
            self._acc += d
            self._buffer.append(d)

        self._last_timestamp = __ts

    @property
    def fps(self) -> float:
        try:
            return len(self._buffer)/self._acc
        except:
            return 0.


class VelarrayDriver:
    def __init__(self, port: int = PORT) -> None:
        rospy.init_node('driver', log_level=rospy.DEBUG)

        # grab the udp socket on which the packets are incoming
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))

        # buffers to store incoming packets / timestamps
        self._buffer: deque[VelodyneDataPacket] = deque()
        self._fpscounter = AverageFrameRate(10)

        # load configuration
        with open(os.path.join(pckg_path, 'config/M1600.yaml')) as fp:
            self._configs = yaml.safe_load(fp)

        # init pointcloud stuff
        self._pointfields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=1*4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=2*4,
                       datatype=PointField.FLOAT32, count=1),
        ]
        self._msg_header = Header(frame_id='velodyne', stamp=rospy.Time.now())

        # publisher
        self._pub = rospy.Publisher('scan', PointCloud2, queue_size=10)

    # def construct_point(self, lf:FiringReturn) -> 'tuple[float, ...]' :
    #     yx = lf.dist * distLSB_ *
    #     x =

    def construct_pointcloud(self) -> PointCloud2:

        points = np.array(
            [[1 for lf in packet.data] for packet in self._buffer]
        )

        return PointCloud2(
            header=self._msg_header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=self._pointfields,
            point_step=len(self._pointfields)*4,
            row_step=(len(self._pointfields)*4 * points.shape[0]),
            data=points.astype(np.float32).tobytes()
        )

    def average_fps(self) -> float:
        '''return the average framerate'''
        return self._fpscounter.fps

    def run(self):
        while (not rospy.is_shutdown()):
            packet = VelodyneDataPacket(self.sock.recv(_MTU))

            if packet.pseqf == 0:
                self._buffer.clear()
                self._msg_header.stamp = rospy.Time.now()
                self._fpscounter.new_timestamp(self._msg_header.stamp)
                rospy.logdebug_throttle(
                    1, f"avg FPS : {self.average_fps():.1f}")

            self._buffer.append(packet)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.sock.close()


def main():
    with VelarrayDriver() as vd:
        vd.run()


if __name__ == '__main__':
    main()
