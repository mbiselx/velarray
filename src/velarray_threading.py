#!/usr/bin/env python
import os
import sys
import yaml
import math
import socket
import typing
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

_datatype_lookup = {
    'B': PointField.UINT8,
    'f': PointField.FLOAT32,
    'd': PointField.FLOAT64,
}
'''trasform from 'struct'-module format characters to PointField codes '''

_datasize_lookup = {
    'B': 1,
    'f': 4,
    'd': 8,
}


class VelarrayDriver:
    def __init__(self, port: int = 2368) -> None:
        '''default port used by the velarray is 2368'''
        rospy.init_node('driver', log_level=rospy.DEBUG)

        # grab the udp socket on which the packets are incoming
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))

        # buffers to store incoming packets / timestamps
        self._pack_buf = deque()
        self._fpscounter = FramerateCounter(10)
        self._losscounter = PacketLossCounter(20)

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

        # pointcloud stuff
        self._point_fields = []
        offset = 0
        for name, type in zip(self.packet_converter.fields, self.packet_converter.structure.format):
            self._point_fields.append(PointField(name=name,
                                                 offset=offset,
                                                 datatype=_datatype_lookup[type],
                                                 count=1))
            offset += _datasize_lookup[type]
        self._point_step = self.packet_converter.structure.size

    def pub(self, packets: 'list[VelodyneDataPacket]'):

        point_data = b''.join(self.packet_converter.packet_to_bytes(
            packet) for packet in packets)
        width = len(point_data)//self._point_step

        pcl = PointCloud2(
            header=self._msg_header,
            height=1,
            width=width,
            is_dense=False,
            is_bigendian=False,
            fields=self._point_fields,
            point_step=self._point_step,
            row_step=len(point_data),
            data=point_data
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

                loss = packet.pseq - self._pack_buf[-1].pseq - 1
                self._losscounter.new_report(loss)

            self._pack_buf.append(packet)

            rospy.logdebug_throttle(
                1, f"avg FPS : {self._fpscounter.avg_fps:.1f}, avg loss : {self._losscounter.avg_loss:.1f}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.sock.close()


def main():
    with VelarrayDriver() as vd:
        vd.run()


if __name__ == '__main__':
    main()
