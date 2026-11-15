#!/usr/bin/env python3
"""汇总恢复实验：同刻 IMU 真值评分、独立阶段时间、资源与导航结果，不丢弃失败批次。"""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


# 复用现有 bag/机械 TF 评分函数，真值只进入离线评分。
def scoring_module():
    path = Path(__file__).with_name('benchmark_rm27_prior_map.py')
    spec = importlib.util.spec_from_file_location('prior_map_score', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# 对每次运行保留全部有效输出样本；前端每个 epoch 只作一次真值刚体对齐。
def summarize(directory, imu_frame):
    from rclpy.serialization import deserialize_message
    from rm27_localization_interfaces.msg import FrontendFrame
    scoring = scoring_module()
    directory = Path(directory)
    data = json.loads((directory/'results.json').read_text())
    times, positions, rotations, base, buffer = scoring.load_truth(directory/'bag')
    unique, indices = np.unique(times, return_index=True)
    positions, rotations = positions[indices], rotations[indices]
    slerp = Slerp(unique, Rotation.from_quat(rotations))
    reader, _ = scoring.open_reader(directory/'bag')
    raw = {}
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic != '/point_lio/backend_frame':
            continue
        frame = deserialize_message(payload, FrontendFrame)
        p, q = frame.odometry.pose.pose.position, frame.odometry.pose.pose.orientation
        stamp = frame.odometry.header.stamp.sec + frame.odometry.header.stamp.nanosec*1e-9
        raw[round(stamp, 8)] = (frame.epoch, scoring.pose_matrix([p.x,p.y,p.z], [q.x,q.y,q.z,q.w]))
    errors, front_errors, aligned = [], [], {}
    tf_missing = 0
    for stamp, *pose in data['estimates']:
        if stamp < unique[0] or stamp > unique[-1]:
            continue
        try:
            base_imu = scoring.transform_at(buffer, base, imu_frame, stamp)
        except Exception:
            tf_missing += 1
            continue
        truth = scoring.pose_matrix([np.interp(stamp, unique, positions[:, i]) for i in range(3)],
                                     slerp([stamp]).as_quat()[0]) @ base_imu
        estimate = scoring.pose_matrix(pose[:3], pose[3:])
        delta = np.linalg.inv(truth) @ estimate
        errors.append([np.linalg.norm(delta[:3, 3]), np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude())])
        original = raw.get(round(stamp, 8))
        if original is not None:
            epoch, odom = original
            if epoch not in aligned:
                aligned[epoch] = truth @ np.linalg.inv(odom)
            front_errors.append(np.linalg.norm((aligned[epoch] @ odom)[:3, 3]-truth[:3, 3]))
    states = data['states']
    changes = [s for i, s in enumerate(states) if i == 0 or
               (s['state'],s['epoch']) != (states[i-1]['state'],states[i-1]['epoch'])]
    initial = next((s for s in states if s['trusted']), None)
    resources = data['resources']
    summary = dict(run=directory.name, output_samples=len(data['estimates']), scored_samples=len(errors),
                   tf_missing=tf_missing, first_trusted_wall=initial['wall'] if initial else None,
                   first_trusted_reason=initial['reason'] if initial else None, goals=data.get('goals',[]),
                   invalid_reset_result=data.get('invalid_reset_result'), state_changes=changes,
                   epochs=sorted(set(s['epoch'] for s in states)),
                   nodes=data.get('nodes'), tf_publishers=data.get('tf_publishers'),
                   cpu_convention='100%=one logical CPU; process only',
                   memory_convention='MiB; backend process RSS and PSS separately')
    if errors:
        e = np.asarray(errors)
        summary.update(position_rmse_m=float(np.sqrt(np.mean(e[:,0]**2))),
                       position_p95_m=float(np.percentile(e[:,0],95)), position_max_m=float(e[:,0].max()),
                       rotation_mean_deg=float(e[:,1].mean()), rotation_max_deg=float(e[:,1].max()))
    if front_errors:
        summary.update(frontend_aligned_samples=len(front_errors),
                       frontend_per_epoch_aligned_rmse_m=float(np.sqrt(np.mean(np.array(front_errors)**2))))
    if resources:
        cpu = np.array([r['cpu'] for r in resources])
        summary.update(cpu_mean=float(cpu.mean()), cpu_p95=float(np.percentile(cpu,95)), cpu_max=float(cpu.max()))
        for name in ['rss','pss']:
            values=np.array([r[name] for r in resources])/1024**2
            summary[name+'_mean_mib']=float(values.mean())
            summary[name+'_max_mib']=float(values.max())
    summary['last_diagnostics']=data['diagnostics'][-1] if data['diagnostics'] else {}
    (directory/'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    return summary


# 支持多个完整实验目录，保持运行身份和失败分母。
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+')
    parser.add_argument('--imu-frame', default='imu_link')
    parser.add_argument('--output', required=True)
    args=parser.parse_args()
    summaries=[summarize(d,args.imu_frame) for d in args.directories]
    Path(args.output).write_text(json.dumps(summaries,indent=2,ensure_ascii=False))
    for row in summaries:
        print(json.dumps({k:v for k,v in row.items() if k not in ['last_diagnostics','state_changes','nodes','tf_publishers']},ensure_ascii=False))


if __name__=='__main__':
    main()
