import json
from pathlib import Path
import numpy as np
import pytest
from survey_protocol import (points_hash,read_ordered_protocol,route_arclength,select_arc_stations,settle_sensor_complete,validate_route_grid)


def test_arc_targets_select_only_existing_supported_vertices_and_keep_closure():
    start=[0,0];route=[[float(x),0]for x in range(1,11)]+[[0,0]]
    supported=np.ones(12,bool);supported[4:7]=False;heights=np.zeros(12)
    stations,missed=select_arc_stations(start,route,supported,heights,2.5)
    assert stations[0]['waypoint_index']==-1 and stations[-1]['waypoint_index']==10
    assert stations[-1]['kind']=='closure' and stations[-1]['xy']==stations[0]['xy']
    assert stations[-1]['id']!=stations[0]['id']
    assert stations[-1]['revisit_of_station_id']==stations[0]['id']
    assert 5.0 in missed
    for row in stations:
        i=row['waypoint_index']+1
        assert supported[i] and row['xy']==[start,*route][i]
    assert route_arclength(start,route)[-1]==20


def fixture_protocol(tmp_path):
    start=[0.,0.];route=[[1.,0.],[1.,1.],[0.,0.]];stations,_=select_arc_stations(start,route,np.ones(4,bool),np.zeros(4),1.)
    source=tmp_path/'source.txt';source.write_text('geometry frozen')
    import hashlib
    data={'schema':'ordered_mapping_survey_v1','preserve_waypoint_order':True,'start_xy':start,'waypoints':route,'ordered_waypoints_sha256':points_hash(route),'length_m':float(route_arclength(start,route)[-1]),'stations':stations,'source_hashes':{str(source):hashlib.sha256(source.read_bytes()).hexdigest()}}
    path=tmp_path/'protocol.json';path.write_text(json.dumps(data));return path,data,source


def test_ordered_protocol_preserves_vertices_and_rejects_reordering_or_source_drift(tmp_path):
    path,data,source=fixture_protocol(tmp_path);loaded=read_ordered_protocol(path,[0,0]);assert loaded['waypoints']==data['waypoints']
    data['waypoints'][0],data['waypoints'][1]=data['waypoints'][1],data['waypoints'][0];path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='order/hash'):read_ordered_protocol(path,[0,0])
    data['waypoints'][0],data['waypoints'][1]=data['waypoints'][1],data['waypoints'][0];path.write_text(json.dumps(data));source.write_text('changed geometry')
    with pytest.raises(ValueError,match='source changed'):read_ordered_protocol(path,[0,0])


def test_unsupported_initial_or_closure_is_not_silently_omitted():
    for mask in [[False,True,True],[True,True,False]]:
        with pytest.raises(ValueError,match='closure stations'):select_arc_stations([0,0],[[1,0],[0,0]],mask,[0,0,0])


def test_grid_validation_checks_supplied_order_without_replanning_or_corner_cut():
    class Planner:
        resolution=1.
        safe=np.ones((3,3),bool)
        @staticmethod
        def cell(xy):return int(xy[1]),int(xy[0])
    p=Planner();route=[[1.5,.5],[1.5,1.5]];before=json.dumps(route);validate_route_grid(p,[.5,.5],route);assert json.dumps(route)==before
    p.safe[0,1]=False
    with pytest.raises(ValueError):validate_route_grid(p,[.5,.5],[[1.5,1.5]])


def test_sensor_settle_is_duration_not_number_of_callbacks_or_command_time():
    assert not settle_sensor_complete(10.,10.,2.5)
    assert not settle_sensor_complete(10.,12.4,2.5)
    assert settle_sensor_complete(10.,12.5,2.5)
    with pytest.raises(ValueError,match='backwards'):settle_sensor_complete(10.,9.9,2.5)
