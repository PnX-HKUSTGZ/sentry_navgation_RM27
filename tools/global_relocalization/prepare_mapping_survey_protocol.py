#!/usr/bin/env python3
"""Prepare ordered source-only station coverage without starting any simulator."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
import yaml
from actual_collision_support import filter_supported_positions
from survey_protocol import points_hash,route_arclength,select_arc_stations


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(source_route,geometry_config,output,spacing=2.5):
    source_route,geometry_config,output=[Path(x).resolve()for x in [source_route,geometry_config,output]]
    if output.exists():raise FileExistsError(str(output))
    original=json.loads(source_route.read_text());start=[-11.7,3.1];route=original['waypoints'];points=np.array([start,*route]);arc=route_arclength(start,route)
    if not math.isclose(float(arc[-1]),original['length_m'],abs_tol=1e-9):raise ValueError('original route length does not match its ordered vertices')
    config=yaml.safe_load(geometry_config.read_text());options=dict(config['collision_support'])
    for key in ['world_path','robot_urdf_path']:options[key]=(geometry_config.parent/options[key]).resolve()
    options['model_roots']=[(geometry_config.parent/x).resolve()for x in options.get('model_roots',[])];options['radial_margin']=.10
    supported,heights,geometry=filter_supported_positions(points,**options)
    stations,missed=select_arc_stations(start,route,supported,heights,spacing)
    source_hashes={**geometry['source_hashes'],str(source_route):sha(source_route),str(geometry_config):sha(geometry_config),str(Path(__file__).resolve()):sha(__file__),
        str(Path(__file__).with_name('survey_protocol.py').resolve()):sha(Path(__file__).with_name('survey_protocol.py'))}
    for field in ['image']:
        path=(geometry_config.parent/config[field]).resolve();source_hashes[str(path)]=sha(path)
    path=(geometry_config.parent/config['ground_elevation']['grid']['image']).resolve();source_hashes[str(path)]=sha(path)
    spacing_actual=np.diff([x['arc_length_m']for x in stations]);turns=2.05;settle=2.5;speed=.3;omega=1.;post=.5
    nominal_transit=float(arc[-1]/speed);nominal_station=len(stations)*(turns*2*np.pi/omega+settle+post)
    # Predeclared operating allowance, not derived from any query/GT error.
    expected_total=1.35*nominal_transit+len(stations)*(turns*2*np.pi/omega+settle+post+1.2)+60.
    baseline_metadata=source_route.parent/'raw_bag/metadata.yaml';bag_rate=None;bag_source={}
    if baseline_metadata.is_file():
        md=yaml.safe_load(baseline_metadata.read_text())['rosbag2_bagfile_information'];seconds=md['duration']['nanoseconds']*1e-9
        paths=[baseline_metadata.parent/x['path']for x in md['files']];total=sum(p.stat().st_size for p in paths);bag_rate=total/seconds
        bag_source={'metadata':str(baseline_metadata),'metadata_sha256':sha(baseline_metadata),'duration_sec':seconds,'actual_bytes':total,'rate_bytes_sec':bag_rate};source_hashes[str(baseline_metadata)]=sha(baseline_metadata)
    result={'schema':'ordered_mapping_survey_v1','preserve_waypoint_order':True,'start_xy':start,'waypoints':route,'ordered_waypoints_sha256':points_hash(route),
        'source_route':str(source_route),'source_route_sha256':sha(source_route),'source_route_waypoint_count':len(route),'length_m':float(arc[-1]),'original_length_m':original['length_m'],'route_length_change_m':float(arc[-1]-original['length_m']),
        'station_selection':{'rule':'nearest supported existing route vertex to each fixed arc target, max arc offset spacing/2, deterministic lower-index tie; initial and explicit closure always retained',
            'spacing_target_m':spacing,'maximum_target_arc_offset_m':spacing/2,'query_or_GT_error_used':False,'no_off_route_relocation':True,'controller_arrival_tolerance_m':.075,'radial_margin_m':.10,
            'radial_margin_reason':'covers 0.075m arrival error plus 0.025m additional margin; full-turn swept robot envelope included'},
        'station_turn_protocol':{'controller_turns':2.05,'complete_sensor_turns_required':2,'fixed_overscan_degrees':18.,'overscan_tail_role':'auxiliary retained raw frames, never stretched into second complete gyro turn'},
        'stations':stations,'station_count_including_initial_and_closure':len(stations),'missed_arc_targets_m':missed,'actual_station_arc_gaps_m':spacing_actual.tolist(),
        'station_height_counts_m':{f'{z:.3f}':sum(round(s['actual_support_height_m'],3)==z for s in stations)for z in sorted(set(round(s['actual_support_height_m'],3)for s in stations))},
        'geometry':geometry,'source_hashes':source_hashes,
        'candidate_route_vertices':[{'waypoint_index':i-1,'xy':xy.tolist(),'arc_length_m':float(arc[i]),'full_turn_supported':bool(supported[i]),'actual_support_height_m':float(heights[i])if supported[i]else None}for i,xy in enumerate(points)],
        'minimum_control':{'survey_terrain':'surveyed','control_source':'simulator','station_scan_turns':2.05,'station_scan_speed_rad_sec':1.,'station_settle_sec':2.5,'rotation_speed_rad_sec':0.,'maximum_translation_speed_m_sec':.3},
        'recommended_control':{'speed_m_sec':speed,'max_estimator_lag_sec':.5,'max_wall_sec':math.ceil(expected_total*1.5)},
        'estimate':{'nominal_transit_sec':nominal_transit,'nominal_station_total_sec':nominal_station,'expected_total_sec_with_motion_and_startup_allowance':expected_total,
            'time_allowance_rule':'1.35x ideal translational transit + ideal turn/settle/post-stop +1.2sec turn overhead per visit +60sec startup/export allowance',
            'baseline_bag_rate_source':bag_source,'expected_bag_GiB':expected_total*bag_rate/1024**3 if bag_rate else None,'bag_disk_budget_GiB_1p5_allowance':expected_total*bag_rate*1.5/1024**3 if bag_rate else None,
            'expected_full_raw_PCD_cache_GiB':expected_total*10*69120*12/1024**3,
            'recommended_total_workspace_disk_budget_GiB':10*math.ceil((expected_total*bag_rate*1.5/1024**3+expected_total*10*69120*12*1.5/1024**3+2)/10)if bag_rate else None},
        'scope':'new source-mapping capture protocol only, not new relocalization validation points; controller GT may navigate simulator but must never enter estimation'}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({k:result[k]for k in ['length_m','route_length_change_m','station_count_including_initial_and_closure','missed_arc_targets_m','station_height_counts_m','estimate']},indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['source_route','geometry_config','output']:p.add_argument(name,type=Path)
    p.add_argument('--spacing',type=float,default=2.5);a=p.parse_args();prepare(a.source_route,a.geometry_config,a.output,a.spacing)


if __name__=='__main__':main()
