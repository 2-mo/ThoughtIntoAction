import json
import socket
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from mcap.reader import make_reader
from robot_control.motor_feedback import decode_feedback, MotorFeedback
from quest_teleop.mcap_io import RecordWriter

TYPES = {1:'EC_A4310', 2:'EC_A4310', 4:'EC_A4310', 5:'DM_J4310', 6:'DM_J4310', 7:'DM_J4310', 8:'DM_J4310'}

class MotorFeedbackTests(unittest.TestCase):
    def test_real_wire_layouts_and_current_units(self):
        ec = decode_feedback(1, bytes.fromhex('207faf7ff8057482'), TYPES)
        self.assertAlmostEqual(ec['current_a'], 0x805*60/4095-30)
        self.assertEqual(ec['temperature_c'], 33)
        self.assertEqual(ec['fault_code'], 0)
        dm = decode_feedback(0, bytes.fromhex('187f9e7ff7ff221c'), TYPES)
        self.assertEqual(dm['motor_id'],8)
        self.assertIsNone(dm['current_a'])
        self.assertEqual(dm['load_unit'],'N*m')
        self.assertEqual(dm['temperature_c'],34)
        self.assertEqual(dm['rotor_temperature_c'],28)
        self.assertIsNone(dm['bus_voltage_v'])
        self.assertEqual(decode_feedback(0,bytes.fromhex('987f9e7ff7ff221c'),TYPES)['fault_code'],9)
        self.assertIsNone(decode_feedback(8,bytes.fromhex('7f9e8000000007ff'),TYPES))
        self.assertIsNone(decode_feedback(0x80000001,bytes(8),TYPES))
        self.assertIsNone(decode_feedback(1,bytes(7),TYPES))

    def test_missing_and_stale_data_never_becomes_zero_or_valid(self):
        with patch('robot_control.motor_feedback.socket.socket', side_effect=OSError('offline')):
            observer=MotorFeedback('offline',[1,2,4,5,6,7],list(TYPES.values())[:6],8,'DM_J4310')
        self.addCleanup(observer.close)
        self.assertTrue(all(not m['valid'] and m['current_a'] is None for m in observer.snapshot()['motors']))
        now=time.monotonic_ns()
        observer.error=None
        observer.latest[1]={**decode_feedback(1,bytes.fromhex('207faf7ff8057482'),TYPES),'host_received_monotonic_ns':now}
        self.assertTrue(observer.snapshot()['motors'][0]['valid'])
        with patch('robot_control.motor_feedback.time.monotonic_ns',return_value=now+300_000_000):
            snapshot=observer.snapshot()
        self.assertEqual(snapshot['motors'][0]['feedback_status'],'stale')
        self.assertIsNone(snapshot['motors'][0]['current_a'])
        self.assertFalse(snapshot['events'][-1]['valid'])

    def test_low_frequency_diagnostics_and_deduplicated_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'test.mcap'
            with path.open('wb') as stream:
                writer=RecordWriter(stream,{'started_monotonic_ns':100, 'started_unix_ns':1000})
                event={'sequence':1,'host_monotonic_ns':110,'motor_id':1,'valid':False}
                for i in range(3):
                    stamp=120+i*600_000_000
                    writer.write_state({'host_monotonic_ns':stamp,'joint_current_a':[1,None],
                        'motor_diagnostics':{'stream_id':'test','host_monotonic_ns':stamp,'motors':[], 'events':[event]}},i)
                self.assertEqual(writer.extra_counts,{'diagnostics':2,'event':1})
                writer.finish()
            with path.open('rb') as stream:
                messages=list(make_reader(stream).iter_messages())
            self.assertEqual(len(messages),6)
            states=[json.loads(m.data) for _,c,m in messages if c.topic=='/robot/state']
            self.assertNotIn('motor_diagnostics',states[0])
            self.assertEqual(states[0]['joint_current_a'],[1,None])

if __name__=='__main__':unittest.main()
