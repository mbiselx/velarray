#!/usr/bin/env python
import os
import sys
import yaml
import math
import typing
import multiprocessing
import multiprocessing.connection

import rospy
import rospkg

import tf2_ros
import tf.transformations

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField, PointCloud2

# messing around with paths to get imports to work
try:
    rospack = rospkg.RosPack()
    pckg_path = rospack.get_path('velarray')
    sys.path.append(pckg_path)
    from src.velarray_datapackets import VelarrayDataPacketConverter
    from src.velarray_receiver import VelarrayReceiveProcess
    from src.utils import FramerateCounter
except Exception as e:
    raise e


# trasform from 'struct'-module format characters to PointField codes / sizes
_datatype_lookup = {
    'B': PointField.UINT8,
    'f': PointField.FLOAT32,
    'd': PointField.FLOAT64,
}

_datasize_lookup = {
    'B': 1,
    'f': 4,
    'd': 8,
}


class VelarrayDriver:
    def __init__(self, port: int = 2368) -> None:
        '''default port used by the velarray is 2368'''
        rospy.init_node('velarray')

        # receiver
        self._receiver = VelarrayReceiveProcess(port)

        # load configuration
        with open(os.path.join(pckg_path, 'config/M1600.yaml')) as fp:
            self._configs = yaml.safe_load(fp)

        # packet converter
        self.packet_converter = VelarrayDataPacketConverter(
            distance_resolution=self._configs['distance_resolution'],
            vert_offsets=[l['vert_correction']
                          for l in self._configs['lasers']]
        )

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

    def transform_publisher(self):
        '''from the tutorial : http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28Python%29'''
        tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='velarray',
            ),
            child_frame_id='velarray_sensor',
        )

        static_transformStamped.transform.translation.x = float(0.0497)
        static_transformStamped.transform.translation.y = float(-0.0388)
        static_transformStamped.transform.translation.z = float(0.0275)

        quat = tf.transformations.quaternion_from_euler(
            0., 0., math.radians(-30))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        tf_broadcaster.sendTransform(static_transformStamped)

    def publish_loop(self, pipe: multiprocessing.connection.Connection, rate: typing.Optional[rospy.Rate] = None):
        '''loop used to decode incoming UDP stream and send it on as a pointcloud'''
        if rate is None:
            rate = rospy.Rate(50)
        fpscounter = FramerateCounter(10)
        fpscounter.start()

        self.transform_publisher()

        # publisher
        msg_header = Header(frame_id='velarray_sensor', stamp=rospy.Time.now())
        pub = rospy.Publisher(
            f'{rospy.get_name()}/points',
            PointCloud2,
            queue_size=10
        )

        # process loop
        while not rospy.is_shutdown():
            if pipe.poll():  # check if anything is there
                frame = pipe.recv_bytes()
                point_data = self.packet_converter.bytes_to_bytes(frame)
                width = len(point_data)//self._point_step
                msg_header.stamp = rospy.Time.now()

                pcl = PointCloud2(
                    header=msg_header,
                    height=1,
                    width=width,
                    is_dense=False,
                    is_bigendian=False,
                    fields=self._point_fields,
                    point_step=self._point_step,
                    row_step=len(point_data),
                    data=point_data
                )
                pub.publish(pcl)
                fpscounter.new_frame()

            else:
                rate.sleep()

            rospy.logdebug_throttle(
                1, f"OUT : FPS={fpscounter.avg_fps:.1f}")

    def run(self):
        '''the main loop of the node'''

        # start the reciever sub-process
        rospy.logdebug("starting receiver process")
        self._receiver.start()

        # run the publish loop as the main process, as it has ROS interfaces
        rospy.logdebug("starting publisher process")
        self.publish_loop(self._receiver.recv_pipe)

        rospy.logdebug("teminating processes")
        if self._receiver.is_alive():
            self._receiver.terminate()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        '''clean up on close'''

        # brutal
        if self._receiver.is_alive():
            self._receiver.kill()


def main():
    with VelarrayDriver() as vd:
        vd.run()


if __name__ == '__main__':
    main()
