#!/usr/bin/env python3
import math
import unittest
from mission_logic import area_points, can_reserve
from survey_logic import make_survey, claim_discoveries, choose_survey_task, search_point, owned_age, inspection_view


class Survey(unittest.TestCase):
    def setUp(self):
        self.survey = make_survey({'bounds': [5, -5, 17, 9]})

    def test_equal_contiguous_halves_and_full_camera_coverage(self):
        s = self.survey
        a, b = s['areas']['uav1'], s['areas']['uav2']
        self.assertEqual((a['bounds'], b['bounds']), ([5., -5., 17., 2.], [5., 2., 17., 9.]))
        self.assertEqual(s['split_axis'], 'y')
        pts = [area_points(a), area_points(b)]
        self.assertGreaterEqual(min(abs(x[1]-y[1]) for x in pts[0] for y in pts[1]), 3.5)
        # Conservative 4:3 footprint covers every sampled terrain point,
        # including outer edges and the shared centre line.
        half = (s['altitude_m']-.1)*math.tan(math.pi/6)*.75*.95
        for i in range(49):
            for j in range(57):
                x, y = 5+i*.25, -5+j*.25
                self.assertTrue(any(max(abs(x-p[0]), abs(y-p[1])) <= half for cells in pts for p in cells))

    def test_split_axis_and_validation_are_atomic_inputs(self):
        s = make_survey({'bounds': [0, 0, 12, 6]})
        self.assertEqual(s['split_axis'], 'x')
        for values in ({'bounds': [0,0,3,3]}, {'bounds': [0,0,30,9]},
                       {'bounds': [5,-5,17,9], 'altitude_m': 2.},
                       {'bounds': [5,-5,17,9], 'split_axis': 'bad'}):
            with self.assertRaises(ValueError): make_survey(values)
        with self.assertRaises(ValueError):make_survey({'bounds': [5,-5,17,9]}, separation=6.5)

    def test_first_observer_claims_persist_and_outside_is_ignored(self):
        fires = [dict(id=1, centre_m=[8,0], first_observer='uav1'),
                 dict(id=2, centre_m=[12,6], first_observer='uav2'),
                 dict(id=3, centre_m=[-10,0], first_observer='uav1')]
        claims = claim_discoveries(fires, {}, self.survey)
        self.assertEqual(claims, {1:'uav1',2:'uav2'})
        self.assertEqual(claim_discoveries(fires, {1:'uav2',2:'uav2'}, self.survey), {})

    def test_transit_or_previous_partition_cannot_claim_an_uninspectable_fire(self):
        survey=make_survey({'bounds':[5,-5,17,9],'split_axis':'x'})
        fire=dict(id=1,centre_m=[8.,0.],boundary_m=[[7.,-1.,0.],[9.,1.,0.]],
                  first_observer='uav2',last_seen_by_uav_s={'uav2':1.})
        self.assertEqual(claim_discoveries([fire],{},survey),{})
        fire['last_seen_by_uav_s']['uav1']=0.
        self.assertEqual(claim_discoveries([fire],{},survey),{1:'uav1'})

    def test_revisit_uses_owners_camera_and_starts_before_deadline(self):
        state = dict(position_m=[8.,-2.,5.], survey_index=1)
        fire = dict(id=1, centre_m=[8.,-2.5], last_seen_age_s=0.,
                    last_seen_by_uav_s={'uav1':19.,'uav2':0.})
        task = choose_survey_task('uav1',state,[fire],{1:'uav1'},set(),self.survey)
        self.assertEqual(task['kind'],'fire')
        self.assertEqual(task['fire_ids'],[1])
        fire['last_seen_by_uav_s']['uav1']=0.
        self.assertEqual(choose_survey_task('uav1',state,[fire],{1:'uav1'},set(),self.survey)['kind'],'search')
        self.assertEqual(choose_survey_task('uav1',state,[fire],{1:'uav2'},set(),self.survey)['kind'],'search')

    def test_fire_task_stays_inside_own_flight_half(self):
        state = dict(position_m=[8.,0.,5.],survey_index=0)
        fire = dict(id=1,centre_m=[8.,2.1],last_seen_by_uav_s={'uav1':30.})
        task=choose_survey_task('uav1',state,[fire],{1:'uav1'},set(),self.survey)
        self.assertLessEqual(task['position'][1], .1+1e-9)
        self.assertEqual(task['fire_ids'],[1])

    def test_coverage_reverses_without_jump_or_losing_index_on_revisit(self):
        area=self.survey['areas']['uav1'];points=area_points(area);n=len(points)
        self.assertEqual(search_point({'survey_index':n},area)[0],points[-2])
        self.assertEqual(search_point({'survey_index':2*n-2},area)[0],points[0])
        state=dict(position_m=points[0],survey_index=4)
        fire=dict(id=1,centre_m=[8.,0.],last_seen_by_uav_s={'uav1':40.})
        choose_survey_task('uav1',state,[fire],{1:'uav1'},set(),self.survey)
        self.assertEqual(state['survey_index'],4)

    def test_infeasible_revisits_do_not_starve_coverage(self):
        state=dict(position_m=[15.,0.,5.],survey_index=0,
                   history=[{'kind':'fire'}])
        fire=dict(id=1,centre_m=[15.,0.],last_seen_by_uav_s={'uav1':40.})
        self.assertEqual(choose_survey_task('uav1',state,[fire],{1:'uav1'},set(),self.survey)['kind'],'search')
        state['history']=[]
        fire['last_seen_by_uav_s']['uav1']=0.
        self.assertEqual(choose_survey_task('uav1',state,[fire],{1:'uav1'},set(),self.survey)['kind'],'search')

    def test_nearby_fires_share_one_inspection_using_measured_boundaries(self):
        a=dict(id=1,centre_m=[8.,-2.5],boundary_m=[[7.5,-3.,0.],[8.5,-2.,0.]])
        b=dict(id=2,centre_m=[8.,0.],boundary_m=[[7.,-1.,0.],[9.,1.,0.]])
        far=dict(id=3,centre_m=[14.,-2.5],boundary_m=[[13.25,-3.25,0.],[14.75,-1.75,0.]])
        target,ids=inspection_view(a,[a,b,far],self.survey['areas']['uav1'])
        self.assertEqual(ids,[1,2])
        self.assertAlmostEqual(target[1],-1.)
        c=dict(id=4,centre_m=[12.,6.],boundary_m=[[10.5,5.,0.],[13.5,7.,0.]])
        f=dict(id=5,centre_m=[15.,6.],boundary_m=[[14.5,5.25,0.],[15.5,6.75,0.]])
        _,ids=inspection_view(c,[c,f],self.survey['areas']['uav2'])
        self.assertEqual(ids,[4])


if __name__=='__main__':unittest.main()
