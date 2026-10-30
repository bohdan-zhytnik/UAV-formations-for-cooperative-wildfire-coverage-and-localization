#!/usr/bin/env python3
"""Exercise the two-drone shared survey in Gazebo; ends landed and paused."""
import argparse
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise-flight',action='store_true')
    parser.add_argument('--resume-flight',action='store_true',help='Both drones already hovering after a controls reload')
    args=parser.parse_args()
    if not args.exercise_flight:parser.error('Pass --exercise-flight to run simulated flight commands')
    rclpy.init()
    node=Node('validate_shared_survey')
    latest,acks={},{}
    report={'checks':[],'max_owner_age_sim_s':{},'active_max_owner_age_sim_s':{},'minimum_horizontal_distance_m':None}
    root=Path(__file__).resolve().parents[1]
    truth=yaml.safe_load((root/'config/fleet/fires.yaml').read_text())['fires']
    durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key,topic in [('mission','/fire_fleet/mission/status'),('map','/fire_fleet/fire_map/status'),('fleet','/fire_fleet/status')]:
        node.create_subscription(String,topic,lambda msg,key=key:latest.__setitem__(key,json.loads(msg.data)),durable)
    def ack(msg):
        data=json.loads(msg.data);acks[data['request_id'],data['uav']]=data
    node.create_subscription(String,'/fire_fleet/mission/config_ack',ack,10)
    for name in ['uav1','uav2']:
        node.create_subscription(String,f'/fire_fleet/{name}/action_ack',ack,durable)
    node.create_subscription(MarkerArray,'/fire_fleet/mission/markers',lambda msg:latest.__setitem__('markers',msg),durable)
    pubs={key:node.create_publisher(String,'/fire_fleet/'+value,10)
          for key,value in [('action','action'),('config','mission/config'),('goal','map_goal')]}
    seen_owners={}
    def state(name):return latest.get('mission',{}).get('uavs',{}).get(name,{})
    def inspect():
        mission=latest.get('mission',{})
        for fire in latest.get('map',{}).get('fires',[]):
            identifier=str(fire['id']);owner=mission.get('assignments',{}).get(identifier)
            if owner:
                assert owner==fire['first_observer'],(owner,fire)
                assert seen_owners.get(identifier,owner)==owner,'Automatic ownership changed'
                seen_owners[identifier]=owner
        for check in mission.get('fire_checks',[]):
            identifier=str(check['id'])
            report['max_owner_age_sim_s'][identifier]=max(report['max_owner_age_sim_s'].get(identifier,0.),check['age_sim_s'])
            if state(check['owner']).get('active'):
                report['active_max_owner_age_sim_s'][identifier]=max(report['active_max_owner_age_sim_s'].get(identifier,0.),check['age_sim_s'])
        a,b=state('uav1'),state('uav2')
        if a.get('armed') and b.get('armed') and a.get('position_m') and b.get('position_m'):
            distance=math.dist(a['position_m'][:2],b['position_m'][:2])
            previous=report['minimum_horizontal_distance_m']
            report['minimum_horizontal_distance_m']=min(distance,previous) if previous is not None else distance
        for name in ['uav1','uav2']:
            s=state(name)
            if s.get('active') and s.get('mode')=='cooperative' and s.get('target'):
                p=s['target']['position'];area=mission['areas'][name];bounds=area['bounds'];inset=area.get('inset_m',0.)
                assert bounds[0]+inset-1e-6 <=p[0]<=bounds[2]-inset+1e-6
                assert bounds[1]+inset-1e-6 <=p[1]<=bounds[3]-inset+1e-6
    def wait(predicate,label,timeout=180.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1);inspect()
            if predicate():
                report['checks'].append(label);print('PASS: '+label,flush=True);return
            if any(state(n).get('phase')=='paused' and state(n).get('reason','').startswith((
                'Assigned fire','No fresh','Inspection travel','Coordinator or hardware',
                'Normal flight','MRS rejected','Mission reference','Mission goal','Simulation time')) for n in ['uav1','uav2']):
                raise AssertionError(json.dumps(latest.get('mission',{})))
        raise AssertionError(label+'; '+json.dumps(latest.get('mission',{})))
    def command(key,name,expect=True,**value):
        request=str(uuid.uuid4())
        pubs[key].publish(String(data=json.dumps(dict(uav=name,request_id=request,**value))))
        wait(lambda:(request,name) in acks,f'{name}: {value} acknowledged',90.)
        assert acks[request,name]['success'] is expect,acks[request,name]
    try:
        wait(lambda:all(state(n).get('position_m') and state(n).get('coordinator_fresh') for n in ['uav1','uav2']),
             'Both mission drivers ready and connected',300.)
        assert all(not state(n)['active'] for n in ['uav1','uav2'])
        if not args.resume_flight:
            assert all(not state(n)['armed'] for n in ['uav1','uav2'])
            wait(lambda:not latest.get('map',{}).get('fires',[]) and 'map' in latest,'Map starts empty; six scene positions are not preloaded')
        survey=dict(bounds=[5.,-5.,17.,9.],split_axis='auto',altitude_m=5.,spacing_m=3.,revisit_sim_s=20.)
        command('config','all',op='set_survey',survey=survey)
        command('config','all',expect=False,op='set_survey',survey=dict(survey,bounds=[0.,0.,1.,1.]))
        wait(lambda:latest['mission'].get('survey',{}).get('bounds')==survey['bounds'],'Invalid replacement leaves the accepted region intact')
        for name in ['uav1','uav2']:
            if not args.resume_flight:command('action',name,action='takeoff')
            wait(lambda name=name:state(name).get('ready'),name+' ready in normal flight',300.)
        for name in ['uav1','uav2']:command('action',name,action='mission_start',mode='cooperative')
        command('config','all',expect=False,op='set_survey',survey=survey)
        wait(lambda:all(s['completed_visits']>=2 for s in latest['mission']['uavs'].values()),
             'Both drones search their own halves with fresh camera inspections',600.)
        wait(lambda:len(latest.get('map',{}).get('fires',[]))==6 and len(seen_owners)==6,
             'All six real thermal targets detected and automatically claimed by first observer',1800.)
        fires=latest['map']['fires']
        errors={}
        for patch in truth:
            candidates=[f for f in fires if math.dist(f['centre_m'],patch['position'])<.45]
            assert len(candidates)==1,(patch,candidates)
            errors[patch['name']]=math.dist(candidates[0]['centre_m'],patch['position'])
        report['truth_errors_m']=errors
        report['discovery_checkpoint']=latest['mission']
        wait(lambda:all(c['visited_cells']==c['total_cells'] for c in latest['mission']['survey']['coverage'].values()),
             'Both complete a full coverage sweep despite fire revisit interruptions',1800.)
        wait(lambda:all(any(h['kind']=='fire' and h['expected_fire_ids'] for h in state(n)['history']) for n in ['uav1','uav2']),
             'Both drones explicitly revisit owned fires and confirm with their own cameras',600.)
        report['patrol_checkpoint']=latest['mission']
        command('action','uav1',action='hover')
        wait(lambda:not state('uav1')['active'],'Manual hover pauses only uav1')
        before=state('uav2')['completed_visits']
        wait(lambda:state('uav2')['completed_visits']>before,'uav2 continues the survey independently',300.)
        command('action','uav2',action='mission_pause')
        for name,point in [('uav1',[0.,-6.,5.]),('uav2',[0.,6.,5.])]:
            pubs['goal'].publish(String(data=json.dumps(dict(uav=name,enabled=True,frame='common_origin',
                x=point[0],y=point[1],altitude_m=point[2],heading=0.))))
            wait(lambda name=name,point=point:state(name).get('position_m') and math.dist(state(name)['position_m'],point)<.4,
                 name+' returns to its pad',600.)
            command('action',name,action='land')
            wait(lambda name=name:not state(name)['armed'],name+' lands and disarms',600.)
        report['success']=True
    except Exception as error:
        report.update(success=False,error=str(error))
        pubs['action'].publish(String(data=json.dumps(dict(uav='uav1',action='hover_all',request_id=str(uuid.uuid4())))))
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        raise
    finally:
        report['final_mission']=latest.get('mission',{})
        report['shared_map']=latest.get('map',{})
        (root/'artifacts/shared_survey_check.json').write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node()
        try:
            if rclpy.ok():rclpy.shutdown()
        except Exception as error:
            if 'rcl_shutdown already called' not in str(error):raise


if __name__=='__main__':main()
