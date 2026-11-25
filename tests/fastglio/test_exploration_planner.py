import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'adapters/ros1_noetic/fast_glio_uav'))
from exploration_planner import ExplorationPlanner,advance_return_route


class ExplorationPlannerTest(unittest.TestCase):
    def test_unknown_region_is_never_a_direct_flight_corridor(self):
        p=ExplorationPlanner(10)
        p.free={(x,y) for x in range(-3,15) for y in range(-4,5)}
        result=p.plan([0,0,10],[100,0,10])
        self.assertEqual(result['reason'],'exploring_frontier')
        self.assertGreater(result['path'][-1][0],8)
        for a,b in zip(result['path'],result['path'][1:]):self.assertTrue(p.known_clear(a,b))
        self.assertFalse(p.known_clear([0,0,10],[100,0,10]))

    def test_astar_routes_around_an_inflated_obstacle(self):
        p=ExplorationPlanner(10,clearance=1.5)
        p.free={(x,y) for x in range(-5,25) for y in range(-12,13)}
        p.occupied={(10,y) for y in range(-5,6)}
        r=p.plan([0,0,10],[20,0,10])
        self.assertEqual(r['reason'],'goal_path')
        self.assertGreater(max(abs(q[1]) for q in r['path']),6)
        for a,b in zip(r['path'],r['path'][1:]):self.assertTrue(p.known_clear(a,b))

    def test_observed_occupied_goal_is_rejected(self):
        p=ExplorationPlanner(10);p.free={(x,y) for x in range(20) for y in range(20)};p.occupied={(12,12)}
        self.assertEqual(p.plan([0,0,10],[12,12,10])['reason'],'goal_occupied')

    def test_diagonal_cannot_cut_between_unknown_corners(self):
        p=ExplorationPlanner(10);p.free={(0,0),(1,1)}
        self.assertEqual(p.plan([.5,.5,10],[1.5,1.5,10])['path'],[])

    def test_beams_clear_only_the_observed_height_slab(self):
        p=ExplorationPlanner(10)
        x=np.linspace(10,40,300)
        ground=np.column_stack((x,np.linspace(-.5,.5,300),np.zeros(300)))
        self.assertTrue(p.update([0,0,10],ground))
        self.assertIn((1,0),p.free)
        self.assertNotIn((30,0),p.free)
        self.assertEqual(len(p.occupied),0)

    def test_wall_returns_inflate_and_do_not_clear_behind_wall(self):
        p=ExplorationPlanner(10)
        wall=np.array([[10,y,z] for y in np.linspace(-6,6,50) for z in np.linspace(8,12,20)])
        p.update([0,0,10],wall);p._refresh()
        self.assertIn((10,0),p.occupied)
        self.assertIn((8,0),p.blocked)
        self.assertNotIn((12,0),p.free)

    def test_return_can_use_previously_observed_free_cells(self):
        p=ExplorationPlanner(10);p.free={(x,y) for x in range(-2,80) for y in range(-4,5)}
        result=p.plan([70,0,10],[0,0,10])
        self.assertEqual(result['reason'],'goal_path')
        self.assertLess(np.linalg.norm(np.array(result['path'][-1])-[0,0,10]),1.e-8)

    def test_missing_points_do_not_invent_free_space(self):
        p=ExplorationPlanner(10)
        self.assertFalse(p.update([0,0,10],np.zeros((10,3))))
        self.assertEqual(p.free,set())

class NavigationRegressionTest(unittest.TestCase):
    def open_grid(self):
        p=ExplorationPlanner(12)
        p.free={(x,y) for x in range(-15,30) for y in range(-15,30)}
        return p

    def test_goal_region_accepts_safe_approach_when_centre_is_occupied(self):
        p=self.open_grid();p.occupied={(15,0)}
        result=p.plan([0,0,12],[15,0,12],goal_radius=5)
        self.assertEqual(result['reason'],'goal_region_path')
        self.assertTrue(result['goal_centre_blocked'])
        self.assertLessEqual(np.linalg.norm(np.asarray(result['path'][-1])[:2]-[15,0]),5)
        self.assertTrue(all(p.known_clear(a,b) for a,b in zip(result['path'],result['path'][1:])))

    def test_goal_region_has_no_success_path_through_occupied_cells(self):
        p=self.open_grid();p.occupied={(x,y) for x in range(10,21) for y in range(-5,6)}
        result=p.plan([0,0,12],[15,0,12],goal_radius=2)
        self.assertNotEqual(result['reason'],'goal_region_path')
        for a,b in zip(result['path'],result['path'][1:]):self.assertTrue(p.known_clear(a,b))

    def test_inflation_recovery_retreats_away_from_obstacle(self):
        p=self.open_grid();p.occupied={(3,0)};p._refresh()
        start=np.array([1.5,.5,12.])
        self.assertIn(p.cell(start),p.blocked)
        r=p.plan(start,[20,0,12])
        self.assertEqual(r['reason'],'recovering_clearance')
        for a,b in zip(r['path'],r['path'][1:]):self.assertTrue(p.recovery_clear(a,b))
        self.assertNotIn(p.cell(r['path'][-1]),p.blocked)
        self.assertGreater(p.obstacle_distance(r['path'][-1]),p.obstacle_distance(start))
        self.assertTrue(p.occupied=={(3,0)})

    def test_recovery_never_opens_occupied_or_unknown_space(self):
        p=self.open_grid();p.occupied={(1,0)}
        self.assertEqual(p.plan([1.5,.5,12],[20,0,12])['reason'],'unsafe_recovery_start')
        p=self.open_grid();p.free={(1,0)};p.occupied={(3,0)}
        self.assertEqual(p.plan([1.5,.5,12],[20,0,12])['reason'],'no_safe_retreat')
        self.assertFalse(p.recovery_clear([1.5,.5,12],[4.5,.5,12]))

    def test_tracking_does_not_shortcut_a_corner(self):
        p=self.open_grid();p.occupied={(4,4)}
        path=[[0,0,12],[8,0,12],[8,8,12]]
        target=p.steering_target([7,0,12],path)
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[1],0.)
        self.assertTrue(p.known_clear([7,0,12],target))
        self.assertIsNone(p.steering_target([4.5,2.5,12],path))

    def test_return_skips_dense_guidance_only_on_verified_straight_corridor(self):
        p=self.open_grid();route=[[x,0,12] for x in [1.5,3,4.5,6,7.5,9]]
        self.assertEqual(p.return_guidance_index([0,0,12],route),4)
        p.occupied={(4,0)};p._dirty=True
        self.assertEqual(p.return_guidance_index([0,0,12],route),0)

    def test_obsolete_blocked_return_goal_is_skipped_but_home_is_retained(self):
        p=self.open_grid();p.occupied={(6,0)}
        route=[np.array([6.,0,12]),np.array([0.,0,12])]
        remaining,reached=advance_return_route([10,0,8],route,True,p)
        self.assertEqual(len(remaining),1);self.assertEqual(remaining[0][0],0)
        self.assertFalse(reached)
        p.occupied.add((0,0));p._dirty=True
        remaining,_=advance_return_route([10,0,8],remaining,True,p)
        self.assertEqual(len(remaining),1)

    def test_active_return_target_is_stable_until_reached(self):
        p=self.open_grid();route=[np.array([x,0,12]) for x in [0,1.5,3,4.5,6,7.5,9,10.5]]
        route,reached=advance_return_route([0,0,10],route,True,p)
        self.assertTrue(reached);self.assertEqual(route[0][0],7.5)
        # LIO z does not govern arrival: the caller supplies barometric readiness.
        stable,_=advance_return_route([1.5,0,10],route,True,p)
        self.assertEqual(stable[0][0],7.5)
        held,reached=advance_return_route([7.5,0,10],route,False,p)
        self.assertFalse(reached);self.assertEqual(held[0][0],7.5)

    def test_subcell_goal_region_can_reach_exact_free_goal(self):
        p=self.open_grid();r=p.plan([0,0,12],[20,0,12],goal_radius=.5)
        self.assertEqual(r['reason'],'goal_region_path')
        self.assertLessEqual(np.linalg.norm(np.array(r['path'][-1])[:2]-[20,0]),.5)

    def test_snapshot_is_immutable_and_height_slice_tracks_actual_sensor(self):
        p=self.open_grid();snap=p.snapshot();p.occupied.add((0,0));p._dirty=True
        self.assertTrue(snap.known_clear([0,0,12],[1,0,12]))
        wall=np.array([[10,y,z] for y in np.linspace(-6,6,30) for z in np.linspace(8.8,11,12)])
        p.update([0,0,10],wall)
        self.assertEqual(p.height,10.)

if __name__=='__main__':unittest.main()
