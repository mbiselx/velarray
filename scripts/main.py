#!/usr/bin/env python
import os
import sys
import yaml
import math
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
    from src.utils import *
except Exception as e:
    raise e

_MTU = 2048
'''maximum transferrable unit'''


class VelarrayDriver:
    def __init__(self, port: int = 2368) -> None:
        '''default port used by the velarray is 2368'''
        rospy.init_node('driver', log_level=rospy.DEBUG)

        # grab the udp socket on which the packets are incoming
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))

        # buffers to store incoming packets / timestamps
        self._pack_buf = deque()
        self._point_buf = deque()
        self._frame_buf = deque()
        self._fpscounter = FramerateCounter(10)

        # load configuration
        with open(os.path.join(pckg_path, 'config/M1600.yaml')) as fp:
            self._configs = yaml.safe_load(fp)

        # packet converter
        self.packet_converter = VelarrayDataPacketConverter(
            distance_resolution=self._configs['distance_resolution'],
            vert_offsets=[l['vert_correction']
                          for l in self._configs['lasers']]
        )

        # publisher
        self._msg_header = Header(frame_id='velarray', stamp=rospy.Time.now())
        self._pub = rospy.Publisher(
            '/velarray/scan', PointCloud2, queue_size=10)

    def pub(self, packets: typing.Iterable[VelodyneDataPacket]):
        points: np.ndarray = np.array(
            [self.packet_converter.convert(packet) for packet in packets],
            dtype=np.float32)

        point_fields = [
            PointField(name=name,
                       offset=i*points.itemsize,
                       datatype=PointField.FLOAT32,
                       count=1)
            for i, name in zip(range(points.shape[2]), ('x', 'y', 'z', 'intensity'))
        ]
        point_step = len(point_fields) * points.itemsize
        row_step = point_step * points.shape[0]

        pcl = PointCloud2(
            header=self._msg_header,
            height=points.shape[1],
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=point_fields,
            point_step=point_step,
            row_step=row_step,
            data=points.tobytes()
        )
        self._pub.publish(pcl)

    def run(self):
        while (not rospy.is_shutdown()):

            packet = VelodyneDataPacket(self.sock.recv(_MTU))

            if len(self._pack_buf) > 0:
                if packet.pseqf < self._pack_buf[-1].pseqf:
                    threading.Thread(
                        target=self.pub,
                        args=(self._pack_buf,)
                    ).start()  # D.A.N.G.E.R.O.U.S
                    self._pack_buf = deque()
                    self._msg_header.stamp = rospy.Time.now()
                    self._fpscounter.new_timestamp(self._msg_header.stamp)
                elif packet.pseq - 1 != self._pack_buf[-1].pseq:
                    rospy.logwarn(
                        f"missing {packet.pseq - self._pack_buf[-1].pseq -1} packets")

            self._pack_buf.append(packet)

            rospy.logdebug_throttle(
                5, f"avg FPS : {self._fpscounter.avg_fps:.1f}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.sock.close()


def main():
    with VelarrayDriver() as vd:
        vd.run()


if __name__ == '__main__':
    main()
