import math
import struct
import ctypes


class Header(ctypes.BigEndianStructure):
    '''definition of the header of a datapacket, according to the Velarray User manual.'''
    _fields_: 'list[tuple[str, type, int]]' = [
        ("ver", ctypes.c_ubyte, 4),
        ("hlen", ctypes.c_ubyte, 4),
        ("nxhdr", ctypes.c_ubyte, 8),
        ("ptype", ctypes.c_ubyte, 4),
        ("tlen", ctypes.c_ubyte, 4),
        ("mic", ctypes.c_ubyte, 8),
        ("pseq", ctypes.c_uint32, 32),
        ("tref", ctypes.c_uint64, 64),
        ("glen", ctypes.c_ubyte, 4),
        ("flen", ctypes.c_ubyte, 4),
        ("dset", ctypes.c_uint8, 8),
        ("iset", ctypes.c_uint16, 16),
    ]
    # due to weird compiler issues, we can't use `ctypes.sizeof(Header)`, because it returns 24 ??
    size = 20


class FiringReturn(ctypes.BigEndianStructure):
    '''
    definition of the single firing return of a datapacket, according to the Velarray User manual.
    There should be 160 firing returns per datapacket.
    '''
    _fields_: 'list[tuple[str, type, int]]' = [
        ("vdir", ctypes.c_ubyte, 1),
        ("hdir", ctypes.c_ubyte, 1),
        ("vdfl", ctypes.c_ushort, 14),
        ("azm", ctypes.c_ushort, 16),
        ("dist", ctypes.c_ushort, 16),
        ("rft", ctypes.c_ubyte, 8),
        ("lcn", ctypes.c_ubyte, 8),
    ]
    size = 8

    # @property
    # def ccw_sweep(self) -> bool:
    #     return bool(self.hdir)

    # @property
    # def up_sweep(self) -> bool:
    #     return bool(self.vdir)

    # @property
    # def vertical_deflection(self) -> int:
    #     return self.vdfl


class Footer(ctypes.BigEndianStructure):
    '''definition of the footer of a datapacket, according to the Velarray User manual.'''
    _fields_: 'list[tuple[str, type, int]]' = [
        ("crc", ctypes.c_ushort, 16),
        ("ac", ctypes.c_ubyte, 8),
        ("pseqf", ctypes.c_ubyte, 8),
    ]
    size = 4


class VelodyneDataPacket:
    LASER_FIRES = 160
    _HEADER_END = Header.size
    _DATA_START = _HEADER_END
    _DATA_END = _DATA_START + LASER_FIRES * FiringReturn.size
    _FOOTER_START = _DATA_END
    _FOOTER_END = _FOOTER_START + Footer.size
    size = _FOOTER_END

    def __init__(self, data: bytes) -> None:

        # parse data into structure to make it easier to retrieve
        self.header = Header.from_buffer_copy(data)
        self.data = [FiringReturn.from_buffer_copy(data, self._DATA_START + i*FiringReturn.size)
                     for i in range(self.LASER_FIRES)]
        self.footer = Footer.from_buffer_copy(data, self._FOOTER_START)

        # TODO : crc check

    @property
    def mic(self) -> int:
        '''model identification code'''
        return self.header.mic

    @property
    def tref(self) -> int:
        '''time reference (truncated to 64 bits)'''
        return self.header.tref

    @property
    def pseq(self) -> int:
        '''payload sequence number'''
        return self.header.pseq

    @property
    def pseqf(self) -> int:
        '''payload sequence number within frame'''
        return self.footer.pseqf

    @staticmethod
    def get_pseqs(raw_packet: bytes) -> 'tuple[int, int]':
        '''return pseq, pseqf for a raw datapacket, without needing to parse it'''
        return int.from_bytes(raw_packet[4:8], 'big'), int(raw_packet[-1])


class VelarrayDataPacketConverter:
    '''convert a datapacket to a list of python-readable numbers'''
    fields = ('x', 'y', 'z', 'distance', 'intensity')
    structure = struct.Struct('ffffB')

    def __init__(self, distance_resolution: float, vert_offsets: 'list[float]') -> None:
        '''
        * `distance_resolution` :   the scaling factor to apply to the incoming distance measurements
        * `vert_offsets`        :   the vertical offsets to apply to each individual laser channel, indexed by channel
        '''
        self.distance_resolution = distance_resolution
        self.vert_offsets = vert_offsets
        self._angular_resolution = math.pi/18000

    def construct_point(self, lf: FiringReturn) -> 'tuple[float | int, ...]':
        # distance, azimuth and elevation
        dist = lf.dist * self.distance_resolution
        if lf.dist == 0:  # invalid point
            return *(len(self.fields)-1)*(math.nan,), 0
        azm = lf.azm * self._angular_resolution
        elv = self.vert_offsets[lf.lcn] + lf.vdfl*self._angular_resolution
        # transform this to cartesian
        xy = dist * math.cos(elv)
        x = xy * math.sin(azm)
        y = xy * math.cos(azm)
        z = dist*math.sin(elv)

        return x, y, z, dist, lf.rft

    def packet_to_list(self, packet: VelodyneDataPacket) -> 'list[tuple[float, ...]]':
        '''return the points as a numpy array'''
        return [tuple(map(float, self.construct_point(lf))) for lf in packet.data]

    def point_to_bytes(self, lf: FiringReturn) -> bytes:
        return self.structure.pack(*self.construct_point(lf))

    def packet_to_bytes(self, packet: VelodyneDataPacket):
        return b''.join(self.point_to_bytes(lf) for lf in packet.data)

    def bytes_to_bytes(self, packets: bytes):
        return b''.join(self.packet_to_bytes(VelodyneDataPacket(packets[i:])) for i in range(0, len(packets), VelodyneDataPacket.size))


# a mini demo
if __name__ == '__main__':
    # read input arguments
    import sys
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port',
                        default=2368, type=int,
                        help="the UDP port to listen to")
    args, _ = parser.parse_known_args(sys.argv)

    # start reading incoming datapackets
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', args.port))
    while True:
        packet = VelodyneDataPacket(sock.recv(2048))
        print(packet.header.mic, packet.header.tref,
              packet.header.pseq, packet.footer.pseqf)
