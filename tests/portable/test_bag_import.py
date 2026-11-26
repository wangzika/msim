import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
import tempfile
import unittest
import numpy as np
from msim.bag_import import point_fields,import_bag
from msim.recording import Recording


class CloudLayoutTest(unittest.TestCase):
    def test_big_endian_organized_rows_preserve_padding_and_time(self):
        data=bytearray(48)
        for offset,values in [(0,[1,2,3,.01]),(24,[4,5,6,.098])]:data[offset:offset+16]=np.array(values,dtype='>f4').tobytes()
        msg=S(fields=[S(name=name,count=1,datatype=7,offset=i*4) for i,name in enumerate(['x','y','z','time'])],
              point_step=16,row_step=24,width=1,height=2,is_bigendian=True,data=data)
        fields=point_fields(msg)
        np.testing.assert_array_equal(fields['x'],[1.,4.]);np.testing.assert_allclose(fields['time'],[.01,.098])


@unittest.skipUnless(importlib.util.find_spec('rosbags'),'Optional bag extra not installed')
class BagIntegrationTest(unittest.TestCase):
    def test_convert_real_ros1_serialization_without_ros_runtime(self):
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores,get_typestore
        store=get_typestore(Stores.ROS1_NOETIC);types=store.types
        V=types['geometry_msgs/msg/Vector3'];Q=types['geometry_msgs/msg/Quaternion']
        Header=types['std_msgs/msg/Header'];Time=types['builtin_interfaces/msg/Time'];Imu=types['sensor_msgs/msg/Imu']
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source.bag'
            with Writer(source) as writer:
                c=writer.add_connection('/imu/data','sensor_msgs/msg/Imu',typestore=store)
                msg=Imu(Header(0,Time(3,7),'imu_link'),Q(0.,0.,0.,1.),np.zeros(9),V(0.,0.,0.),np.zeros(9),V(0.,0.,9.8),np.zeros(9))
                writer.write(c,3_200_000_007,store.serialize_ros1(msg,c.msgtype))
            import_bag(source,root/'portable')
            events=list(Recording(root/'portable').events())
            self.assertEqual(len(events),1);self.assertEqual(events[0].stamp_ns,3_000_000_007)
            self.assertEqual(events[0].arrival_ns,3_200_000_007)
            self.assertEqual(events[0].data['acceleration_mps2'],[0.,0.,9.8])

if __name__=='__main__':unittest.main()
