import importlib,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from control import CommandGate,transform_matrix

class ControlTests(unittest.TestCase):
    def ready_gate(self):
        g=CommandGate();g.execute=True;g.front=2;g.clearance=2
        g.fresh={k:10. for k in ['scan','odom','tf','map','camera']};g.propose(.1,.2,10.)
        return g
    def test_no_motion_without_execute(self):
        g=self.ready_gate();g.execute=False;self.assertEqual(g.output(10.),(0,0))
    def test_command_timeout_stops(self):
        self.assertEqual(self.ready_gate().output(10.6),(0,0))
    def test_each_missing_sensor_stops(self):
        for key in ['scan','odom','tf','map','camera']:
            g=self.ready_gate();del g.fresh[key];self.assertEqual(g.output(10.),(0,0),key)
    def test_forward_only_and_speed_clamps(self):
        g=self.ready_gate();g.propose(5,5,10);self.assertEqual(g.output(10),(.18,.5))
        g.propose(-1,-5,10);self.assertEqual(g.output(10),(0,-.5))
    def test_nonfinite_command_stops(self):
        g=self.ready_gate();g.propose(float('nan'),1,10);self.assertEqual(g.output(10),(0,0))
    def test_lidar_blocks_forward_and_close_rotation(self):
        g=self.ready_gate();g.front=.3;self.assertEqual(g.output(10),(0,.2))
        g.clearance=.1;self.assertEqual(g.output(10),(0,0))
    def test_transform_rotates_and_translates(self):
        T=transform_matrix(NS(x=1,y=2,z=3),NS(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
        np.testing.assert_allclose(T@np.array([1,0,0,1]),[1,3,3,1],atol=1e-7)
    def test_invalid_transform_rejected(self):
        with self.assertRaises(ValueError):transform_matrix(NS(x=0,y=0,z=0),NS(x=0,y=0,z=0,w=0))
    def test_core_import_does_not_require_gazebo(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'sim/gazebo/native_sim'))
        original=__import__
        def checked(name,*a,**kw):
            if name=='gz' or name.startswith('gz.'):raise AssertionError('Gazebo dependency on real host')
            return original(name,*a,**kw)
        with patch('builtins.__import__',side_effect=checked):
            import mission
            importlib.reload(mission)

if __name__=='__main__':unittest.main()
