import sys,math,unittest
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'runtime'))
from heading_math import path_concentration,validate_zones,position_error,route_pose
class Contracts(unittest.TestCase):
 def setUp(self):
  self.zones=yaml.safe_load((ROOT/'scenes/S01_subt_heading/scene_semantics.yaml').read_text())['zones']
 def test_three_zones(self):
  for y0,y1,want in [(0,80,5.75),(80,0,5.75),(71,79,20),(56,60,10),(1,10,2)]:
   for bounds in [None,validate_zones(self.zones)]:
    got=path_concentration([0,y0,1.934],[0,y1,1.934],self.zones,prepared_bounds=bounds)
    self.assertAlmostEqual(got['path_average_concentration_mg_m3'],want)
 def test_background_and_boundary(self):
  self.assertEqual(path_concentration([0,-10,1],[0,10,1],self.zones)['path_average_concentration_mg_m3'],1)
  self.assertEqual(path_concentration([0,70,1],[0,80,1],self.zones)['path_average_concentration_mg_m3'],20)
 def test_invalid_paths(self):
  for a,b in [([0,0,1],[0,0,1]),([0,0,1],[0,math.inf,1])]:
   with self.assertRaises(ValueError):path_concentration(a,b,self.zones)
  with self.assertRaises(ValueError):validate_zones(self.zones+[dict(self.zones[0],zone_id='overlap')])
 def test_position_contract(self):
  truth=dict(frame_id='map',stamp_s=1.,xyz_m=[0.,0.,0.])
  self.assertEqual(position_error(dict(truth,xyz_m=[3.,4.,0.]),truth)['position_error_m'],5)
  for update in [dict(stamp_s=1.1),dict(stamp_s=math.nan),dict(frame_id='world'),dict(xyz_m=[math.nan,0,0])]:
   with self.assertRaises(ValueError):position_error(dict(truth,**update),truth)
 def test_route_modes(self):
  route=yaml.safe_load((ROOT/'scenes/S01_subt_heading/demo_route.yaml').read_text())
  for t,want in [(0,(68,'IDLE')),(7,(69,'TRAVEL')),(12,(70,'CUTTING_SIMULATED'))]:self.assertEqual(route_pose(t,route),want)
if __name__=='__main__':unittest.main(verbosity=2)
