#!/usr/bin/env python
import sys
import socket
import multiprocessing
import multiprocessing.connection

import rospy
import rospkg

# messing around with paths to get imports to work
try:
    rospack = rospkg.RosPack()
    pckg_path = rospack.get_path('velarray')
    sys.path.append(pckg_path)
    from src.velarray_datapackets import VelodyneDataPacket
    from src.utils import PacketLossCounter, FramerateCounter
except Exception as e:
    raise e

_MTU = 2048
'''maximum transferrable unit'''


class VelarrayReceiveProcess(multiprocessing.Process):
    '''sub-process to be able to receive data without blocking the pointclound calculations'''

    def __init__(self, port: int = 2368) -> None:

        # init super
        super().__init__(
            name=f'{rospy.get_name()}/receiver',
            target=self.recv_loop,
            daemon=True)

        # create the pipe used to communicate
        self.recv_pipe, self.send_pipe = multiprocessing.Pipe(duplex=False)

        # grab the udp socket on which the packets are incoming
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(1.)
        self._sock.bind(('', port))

    def recv_loop(self):
        '''loop used to receive incoming UDP stream'''
        packets: list[bytes] = []
        losscounter = PacketLossCounter(2000)
        fpscounter = FramerateCounter(10)
        fpscounter.start()

        # track incoming packet sequences
        last_pseq, last_pseqf = -1, -1

        # main loop
        while not rospy.is_shutdown():
            try:
                packet = self._sock.recv(_MTU)
            except socket.timeout:
                rospy.logwarn_throttle(
                    5, f"{rospy.get_name()}: sensor recieve timeout")
                continue

            pseq, pseqf = VelodyneDataPacket.get_pseqs(packet)

            if len(packets) > 0:
                if pseqf < last_pseqf:  # a new frame has started
                    self.send_pipe.send_bytes(b''.join(packets))
                    packets = []
                    fpscounter.new_frame()

                loss = pseq - last_pseq - 1
                losscounter.new_report(loss)

            last_pseq, last_pseqf = pseq, pseqf
            packets.append(packet)

            rospy.logdebug_throttle(
                2, f"IN  : FPS={fpscounter.avg_fps:.1f}, loss={losscounter.avg_loss:.0f}")

    def close(self) -> None:
        self._sock.close()
        return super().close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        '''clean up on close'''
        self.close()
