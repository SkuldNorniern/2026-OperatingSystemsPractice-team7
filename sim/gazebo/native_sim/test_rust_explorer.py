import math,unittest
import numpy as np
from mission import Grid,Mission
from rust_explorer import RustExplorer
import test_mission

class RustExplorerTests(unittest.TestCase):
    def setUp(self):self.explorer=RustExplorer()
    def tearDown(self):self.explorer.close()
    def partial(self):
        g=Grid();g.n=80;g.origin=(0.,0.);g.log=np.zeros((80,80),np.float32)
        g.log[10:60,10:60]=-4;g.log[5:10,5:65]=4;g.log[5:65,5:10]=4
        return g
    def test_frontier_is_selected_from_partial_map(self):
        g=self.partial();goals=self.explorer.goals(g,(3,3,0),[])
        self.assertTrue(goals);self.assertEqual(goals[0]['kind'],'frontier')
        for goal in goals:
            x,y=g.cell(goal['x'],goal['y']);self.assertLess(g.log[y,x],-.4)
    def test_blacklisted_frontier_goals_are_excluded(self):
        g=self.partial();first=self.explorer.goals(g,(3,3,0),[])[0]
        goals=self.explorer.goals(g,(3,3,0),[(first['x'],first['y'])])
        self.assertTrue(all(math.hypot(x['x']-first['x'],x['y']-first['y'])>=.7 for x in goals))
    def test_camera_coverage_after_map_is_known(self):
        g=Grid();g.n=80;g.origin=(0.,0.);g.log=np.full((80,80),-4,np.float32)
        g.log[:4,:]=4;g.log[-4:,:]=4;g.log[:,:4]=4;g.log[:,-4:]=4
        goals=self.explorer.goals(g,(4,4,0),[])
        self.assertTrue(goals);self.assertTrue(all(g['kind']=='coverage' for g in goals))
    def test_known_only_planning_does_not_cross_unknown(self):
        g=self.partial();self.assertFalse(g.plan((3,3),(7,3),None,known_only=True))
    def test_v2_can_finish_after_no_remaining_goals(self):
        m=test_mission.MissionTests().stub();m.version='v2';m.explorer=type('Empty',(),{'goals':lambda *a:[]})()
        m.v2_blacklist=[];m.v2_views=[];m.v2_candidates=[];m.v2_update=-100;m.v2_empty=0
        m.state='V2_SELECT';m.command=lambda *a:None
        m.v2_tick((0,0,0),0);m.v2_tick((0,0,0),1);m.v2_tick((0,0,0),2)
        self.assertEqual(m.state,'DONE');self.assertEqual(m.status,'completed_with_missing_markers')
if __name__=='__main__':unittest.main()
