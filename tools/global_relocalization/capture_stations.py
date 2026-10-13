#!/usr/bin/env python3
"""Map preparation with simulated raw LiDAR at known stationary poses.
Repositioning is offline map acquisition only. Online queries use real cmd_vel.
"""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
from batch_run import reset_pose,stop,launch,ROOT

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--stations',required=True)
p.add_argument('--output',required=True)
p.add_argument('--dwell-wall',type=float,default=2.)
a=p.parse_args();output=Path(a.output).resolve()
if output.exists():
 raise SystemExit('Output exists; choose a new directory')
station_manifest=json.loads(Path(a.stations).read_text());stations=station_manifest['trials']
log=output.parent/(output.name+'.collector.log');log.parent.mkdir(parents=True,exist_ok=True)
collector=launch(['/usr/bin/python3',str(ROOT/'src/global_relocalization/scripts/capture_map.py'),'--output',str(output),
 '--stationary','--duration','3600','--wall-timeout','1800','--scan-stride','1'],log)
commands=[];complete=False
try:
 time.sleep(3)
 for station in stations:
  command,result=reset_pose(station['requested_pose'])
  commands.append({'station':station,'command':command,'result':result,'wall_time':time.time()})
  time.sleep(a.dwell_wall)
  print(f"captured station {station['trial_id']}",flush=True)
 complete=True
finally:
 stop(collector)
 manifest_path=output/'manifest.json'
 if manifest_path.exists():
  manifest=json.loads(manifest_path.read_text())
  manifest.update(capture_method='discontinuous_stationary_simulated_lidar_with_GT_pose_and_actual_TF',
   capture_complete=complete,station_seed=station_manifest['seed'],station_source=str(Path(a.stations).resolve()),
   station_count=len(commands),requested_station_count=len(stations),station_dwell_wall_sec=a.dwell_wall,
   stations=commands,reason='all stationary acquisition stations completed' if complete else 'station acquisition interrupted')
  manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
 if not complete:
  raise SystemExit(2)
