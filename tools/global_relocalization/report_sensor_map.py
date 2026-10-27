#!/usr/bin/env python3
"""Create exportable figures and a local HTML report from measured trial results."""
import argparse
import base64
import csv
from collections import Counter
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evaluation import POSE_CRITERIA


def holdout_status(frozen, summary, audit=None, termination=None):
    """Return an explicit acceptance state; legacy or interrupted runs never pass."""
    fresh = (frozen.get("schema_version", 0) >= 2
             and bool(frozen.get("historical_validation"))
             and frozen.get("trial_source", {}).get("kind") in ("generated", "explicit"))
    complete = summary.get("planned", 0) > 0 and summary.get("completed") == summary.get("planned")
    no_termination = termination is None
    integrity = bool(audit and audit.get("integrity_passed"))
    eligible = bool(audit and audit.get("acceptance_eligible"))
    passed = (fresh and complete and no_termination and integrity and eligible
              and not summary.get("regression_mode", False)
              and bool(summary.get("acceptance_passed"))
              and bool(audit.get("acceptance_passed")))
    if summary.get("regression_mode"):
        status = "regression: 不具备验收资格"
    elif not fresh:
        status = "legacy: 缺少 holdout freshness metadata"
    elif not complete or not no_termination:
        status = "未完成：不得判为通过"
    elif not integrity or not eligible:
        status = "验收证据未核实"
    else:
        status = "通过" if passed else "未通过"
    source = frozen.get("trial_source", {})
    generation = frozen.get("generation_metadata", {})
    validation = frozen.get("historical_validation", {})
    source_hashes = (audit or {}).get("frozen_input_sources", [])
    runtime_changed = [item for item in source_hashes if not item.get("runtime_matches_frozen")]
    return {"passed": passed, "status": status, "fresh": fresh, "complete": complete,
            "integrity_verified": integrity, "acceptance_eligible": eligible,
            "profile": frozen.get("evaluation_profile", summary.get("evaluation_profile", "ideal")),
            "seed": source.get("seed", generation.get("seed")),
            "source_kind": source.get("kind", "legacy/unknown"),
            "sampling_geometry_sha256": generation.get("map_sha256"),
            "history_sources": validation.get("history_source_count"),
            "history_poses": validation.get("history_pose_count"),
            "minimum_history_distance_m": (audit or {}).get("freshness", {}).get("minimum_historical_distance_m"),
            "minimum_batch_distance_m": (audit or {}).get("freshness", {}).get("minimum_batch_distance_m"),
            "minimum_clearance_m": (audit or {}).get("freshness", {}).get("minimum_clearance_m"),
            "minimum_clearance_verified": bool((audit or {}).get("freshness", {}).get(
                "minimum_clearance_verified", False)),
            "generated_trials_reproduced": (audit or {}).get("freshness", {}).get(
                "generated_trials_reproduced"),
            "runtime_sources_changed": len(runtime_changed),
            "runtime_source_hashes": runtime_changed}


def altitude_strata(frozen, results, profile):
    """Summarize every planned trial by requested base height; failures stay in denominator."""
    position_limit, angle_limit = POSE_CRITERIA[profile]
    by_id = {row.get("trial_id"): row for row in results}
    groups = {}
    for trial in frozen.get("trials", []):
        pose = trial.get("requested_pose", {})
        try:
            z = float(pose["z"])
        except (KeyError, TypeError, ValueError):
            z = float("nan")
        key = f"{z:.2f}" if np.isfinite(z) else "未知"
        group = groups.setdefault(key, {"planned": 0, "strict": 0, "profile": 0, "fast": 0, "completed": 0})
        group["planned"] += 1
        row = by_id.get(trial.get("trial_id"))
        if row is None:
            continue
        group["completed"] += 1
        accepted = row.get("accepted")
        if isinstance(accepted, str):
            accepted = accepted.strip().lower() in ("true", "1", "yes")
        if not accepted:
            continue
        try:
            distance = float(row["position_error_m"])
            angle = float(row["orientation_error_deg"])
        except (KeyError, TypeError, ValueError):
            continue
        if not np.isfinite([distance, angle]).all():
            continue
        strict = distance <= 0.10 and angle <= 2.0
        selected = distance <= position_limit and angle <= angle_limit
        group["strict"] += int(strict)
        group["profile"] += int(selected)
        try:
            latency = float(row.get("trigger_to_pose_sec"))
        except (TypeError, ValueError):
            latency = float("inf")
        group["fast"] += int(selected and 0 <= latency <= 1.0)
    return groups


def outcome_metrics(results, planned):
    """Count strict, ideal, and sensor-map correctness and timed joint success."""
    metrics = {"strict": 0, "ideal": 0, "ideal_fast": 0,
               "sensor_map": 0, "sensor_map_fast": 0}
    for row in results:
        accepted = row.get("accepted")
        if isinstance(accepted, str):
            accepted = accepted.strip().lower() in ("true", "1", "yes")
        if not accepted:
            continue
        try:
            distance = float(row["position_error_m"])
            angle = float(row["orientation_error_deg"])
            latency = float(row.get("trigger_to_pose_sec"))
        except (KeyError, TypeError, ValueError):
            continue
        if not np.isfinite([distance, angle]).all():
            continue
        strict = distance <= .10 and angle <= 2.0
        ideal = distance <= POSE_CRITERIA["ideal"][0] and angle <= POSE_CRITERIA["ideal"][1]
        sensor = distance <= POSE_CRITERIA["sensor_map"][0] and angle <= POSE_CRITERIA["sensor_map"][1]
        metrics["strict"] += int(strict)
        metrics["ideal"] += int(ideal)
        metrics["ideal_fast"] += int(ideal and 0 <= latency <= 1.0)
        metrics["sensor_map"] += int(sensor)
        metrics["sensor_map_fast"] += int(sensor and 0 <= latency <= 1.0)
    metrics["planned"] = planned
    return metrics


def failure_reasons_for_profile(results, profile):
    """Classify failures against the selected pose and latency criteria."""
    distance_limit, angle_limit = POSE_CRITERIA[profile]
    reasons = Counter()
    for row in results:
        accepted = row.get("accepted")
        if isinstance(accepted, str):
            accepted = accepted.strip().lower() in ("true", "1", "yes")
        if not accepted:
            reasons[row.get("reason") or "no algorithm output"] += 1
            continue
        try:
            distance = float(row["position_error_m"])
            angle = float(row["orientation_error_deg"])
            latency = float(row.get("trigger_to_pose_sec"))
        except (KeyError, TypeError, ValueError):
            reasons["accepted output missing valid GT score"] += 1
            continue
        if not np.isfinite([distance, angle]).all():
            reasons["accepted output missing valid GT score"] += 1
        elif distance > distance_limit or angle > angle_limit:
            reasons[f"accepted pose exceeds {profile} pose limits"] += 1
        elif not 0 <= latency <= 1.0:
            reasons["pose-correct output exceeds 1 s trigger-to-pose limit"] += 1
    return dict(reasons)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", required=True, type=Path)
    parser.add_argument("--map-dir", required=True, type=Path)
    a = parser.parse_args()
    summary = json.loads((a.batch / "summary.json").read_text())
    manifest = json.loads((a.map_dir / "manifest.json").read_text())
    termination_path = a.batch / "termination.json"
    termination_note = ""
    termination = None
    stopped_early = None
    if termination_path.exists():
        termination = json.loads(termination_path.read_text())
        termination_note = (f"<p><strong>已按用户要求停止；这是部分测试记录。</strong>"
                            f"在线检查点为 {termination['completed_checkpoint_trials']}/{summary['planned']} 组，"
                            f"另有 {termination['uncompleted_trials']} 组未完成。"
                            "基础设施异常、数据过期与算法拒绝分别统计，不能将全部未通过项归因于算法。"
                            "当前先验地图存在已证实的建图发散，不作为合格地图使用；后续工作转为排查建图。</p>")
    stopped_path = a.batch / "stopped_early.json"
    if stopped_path.exists():
        stopped_early = json.loads(stopped_path.read_text())
        termination_note += (f"<p><strong>批次由外部提前停止；不得作为完成的holdout验收。</strong>"
                             f"原因：{html.escape(stopped_early.get('reason', '未注明'))}。"
                             f"截至停止有 {stopped_early.get('completed_results', summary.get('completed', 0))}/"
                             f"{summary.get('planned', 0)} 组结果文件，尚有未执行试次。"
                             f"中断试次：{html.escape(str(stopped_early.get('interrupted_trial', {}).get('trial_id', '无'))) }。</p>")
    frozen_trials = json.loads((a.batch / "trials.json").read_text())
    trials = list(csv.DictReader((a.batch / "results.csv").open()))
    outcome = outcome_metrics(trials, summary.get("planned", 0))
    audit_path = a.batch / "integrity_audit.json"
    integrity_audit = json.loads(audit_path.read_text()) if audit_path.exists() else None
    status = holdout_status(frozen_trials, summary, integrity_audit, termination or stopped_early)
    status["localization_map_sha256"] = manifest.get("map_sha256")
    profile = status["profile"]
    profile_correct = summary.get("pose_correct", 0)
    profile_fast = summary.get("pose_correct_and_fast", 0)
    runtime_source_note = ""
    if status["runtime_source_hashes"]:
        entries = "".join(
            f"<li><code>{html.escape(item['path'])}</code><br>冻结 {html.escape(str(item['frozen_sha256']))}<br>"
            f"运行时 {html.escape(str(item['runtime_sha256']))}</li>"
            for item in status["runtime_source_hashes"])
        runtime_source_note = (f"<details><summary>当前运行时文件与冻结源不同：{status['runtime_sources_changed']}项</summary>"
                               f"<ul>{entries}</ul></details>")
    acceptance_panel = (
        f"<h2>验收状态：{html.escape(status['status'])}</h2>"
        f"<p>holdout来源 {html.escape(str(status['source_kind']))}；profile {html.escape(profile)}；"
        f"seed {html.escape(str(status['seed']))}；完成 {summary.get('completed', 0)}/{summary.get('planned', 0)}。"
        f"严格 0.10 m / 2°：{outcome['strict']}/{outcome['planned']}；"
        f"ideal 精度 {outcome['ideal']}/{outcome['planned']}，且 ≤1 s："
        f"{outcome['ideal_fast']}/{outcome['planned']}；"
        f"sensor_map 精度 {outcome['sensor_map']}/{outcome['planned']}，且 ≤1 s："
        f"{outcome['sensor_map_fast']}/{outcome['planned']}；"
        f"{html.escape(profile)} 精度界限 {POSE_CRITERIA[profile][0]:.2f} m / {POSE_CRITERIA[profile][1]:g}°："
        f"{profile_correct}/{summary.get('completed', 0)}；精度且 ≤1 s："
        f"{profile_fast}/{summary.get('completed', 0)}。"
        f"regression={bool(summary.get('regression_mode'))}；冻结 freshness={status['fresh']}；"
        f"integrity verified={status['integrity_verified']}；当前源文件与冻结hash不同={status['runtime_sources_changed']}项。"
        f"历史源 {status['history_sources']} 个 / {status['history_poses']} 个位置；"
        f"冻结历史最小间距 {status['minimum_history_distance_m']} m；"
        f"批内最小间距 {status['minimum_batch_distance_m']} m。"
        f"地图净空 {status['minimum_clearance_m']} m，几何/seed重放通过={status['minimum_clearance_verified']}。"
        f"采样占据/高程/碰撞几何复合SHA256 <code>{html.escape(str(status['sampling_geometry_sha256']))}</code>；"
        f"定位使用的map.pcd SHA256 <code>{html.escape(str(status['localization_map_sha256']))}</code>。</p>{runtime_source_note}")
    mapping = json.loads((a.map_dir / "mapping_summary.json").read_text())
    full_slam = manifest.get("optimized_graph_used_for_3d_fusion", False)
    method_name = "Point-LIO + SLAM Toolbox" if full_slam else "Point-LIO"
    control_note = ("独立仿真测试控制器读取真值来完成固定采集路线；真值不输入建图或点云融合。"
                    if manifest.get("ground_truth_used_for_control") else "建图和控制没有使用真值。")
    method_note = (f"<p>SLAM Toolbox 已启用扫描匹配、回环检测和 map→odom 发布。"
                   f"三维点云按最终序列化位姿图的 {mapping['graph_nodes']} 个节点、{mapping['graph_edges']} 条边进行重建，"
                   "历史帧采用最终优化位姿，而非直接使用在线 Point-LIO 累积地图。"
                   "SLAM Toolbox 修正 XY 和航向，Z、横滚和俯仰保留 Point-LIO 估计。"
                   "原生 .posegraph/.data、优化节点时间戳、二维栅格地图和逐帧三维点云均保留。</p>"
                   if full_slam else "")
    graph_audit_path = a.map_dir / "graph_correction_audit.json"
    if full_slam and graph_audit_path.exists():
        graph_audit = json.loads(graph_audit_path.read_text())
        method_note += (f"<p>最终图与同时间戳在线节点对照：{graph_audit['online_final_matched_nodes']} 个节点匹配，"
                        f"{graph_audit['historical_nodes_updated']} 个历史节点出现回写变化。"
                        "回环检测开关启用不代表本轮成功闭环；非相邻图边也不能单独证明回环成立。</p>")
    mapping_failure_path = a.map_dir / "mapping_failure_analysis.json"
    if mapping_failure_path.exists():
        window = json.loads(mapping_failure_path.read_text())
        lidar = window["sensor_streams"]["/livox/lidar"]
        imu = window["sensor_streams"]["/livox/imu"]
        method_note += (f"<p>原始 Point-LIO 最大相邻位姿跳变 {window['largest_raw_pointlio_step_m']:.6f} m，"
                        f"间隔 {window['largest_step_dt_sec']:.6f} s。异常窗口雷达最大头时间间隔 "
                        f"{lidar['max_header_gap_sec']:.3f} s，IMU {imu['max_header_gap_sec']:.3f} s；"
                        f"真值高度变化范围 {window['truth_xyz_extent_m'][2]:.3g} m。"
                        "因此三维异常已出现在 SLAM Toolbox 平面修正之前；上述证据不能单独确定数值发散的唯一原因。"
                        "逐帧证据见 mapping_failure_analysis.json 和原始 bag。</p>")
    params = __import__("yaml").safe_load((a.map_dir / "mapping_params.yaml").read_text())["point_lio"]["ros__parameters"]
    context_path = a.map_dir.parent / "source_capture_context.json"
    source_note = ""
    if context_path.exists():
        context = json.loads(context_path.read_text())
        source_note = (f"<p>原始采集路线完成 {context['completed_waypoints']}/{context['planned_waypoints']} 个路径点后停车，"
                       "保留了全部已采集数据，包括末段车辆倾斜。此处重建完成指完整传感器包重放完成。"
                       "完整轨迹存在累积漂移，批测保留由此产生的拒绝和误定位，未根据定位结果筛选地图或删除测试项。</p>")
    analysis_note = ""
    analysis_path = a.batch / "failure_analysis.json"
    if analysis_path.exists():
        analysis = json.loads(analysis_path.read_text())
        labels = {"strict_correct": "严格正确", "false_accept_against_gt": "输出位姿但真值评分不合格",
                  "global_confirmation_rejection": "全局独立确认拒绝", "raw_frontend_rejection": "局部前端拒绝",
                  "global_search_or_state_rejection": "全局检索或状态流程拒绝",
                  "infrastructure_or_client_exception": "基础设施或客户端异常", "accepted_without_valid_gt": "输出缺少有效真值"}
        labels["sensor_or_tf_staleness"] = "传感器/里程计/TF 数据过期"
        analysis_note = "<h2>按结果阶段分类</h2><ul>" + "".join(
            f"<li>{labels.get(k, html.escape(k))}：{v}</li>" for k, v in analysis["category_counts"].items()) + "</ul>"
        quality = analysis["false_accept_fitness_m"]
        if quality["count"]:
            analysis_note += f"<p>误定位样本的配准 fitness 均值为 {quality['mean']:.6f} m。低点云残差与地图坐标偏差可以同时出现；这项阶段分类不单独证明每个失败的唯一原因。逐组证据见 failure_analysis.json。</p>"
    errors = json.loads((a.map_dir / "mapping_errors.json").read_text())
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout="constrained")
    pcd = (a.map_dir / "map.pcd").read_bytes().split(b"DATA binary\n", 1)[1]
    all_points = np.frombuffer(pcd, dtype="<f4").reshape(-1, 3)
    points = all_points
    points = points[(points[:, 2] > .15) & (points[:, 0] < 0)]
    points = points[::max(1, len(points) // 20000)]
    axes[0, 0].scatter(points[:, 0], points[:, 1], s=.3, color="gray", alpha=.3, rasterized=True)
    truth = np.array([r["truth_matrix"] for r in errors])
    estimate = np.array([r["estimated_matrix"] for r in errors])
    axes[0, 0].plot(truth[:, 0, 3], truth[:, 1, 3], label="Gazebo ground truth", linewidth=1.5)
    axes[0, 0].plot(estimate[:, 0, 3], estimate[:, 1, 3], label=method_name + " (single start alignment)", linewidth=1)
    axes[0, 0].set(title="Continuous mapping trajectory", xlabel="World x (m)", ylabel="World y (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].legend(fontsize=8)
    valid = [r for r in trials if r["position_error_m"]]
    if valid:
        plot = axes[0, 1].scatter([float(r["truth_x"]) for r in valid], [float(r["truth_y"]) for r in valid],
                                  c=[float(r["position_error_m"]) for r in valid], cmap="viridis", s=35)
        fig.colorbar(plot, ax=axes[0, 1], label="3D position error (m)")
    failed = [r for r in trials if not r["position_error_m"]]
    requested = {t["trial_id"]: t["requested_pose"] for t in frozen_trials["trials"]}
    if failed:
        axes[0, 1].scatter([requested[r["trial_id"]]["x"] for r in failed],
                            [requested[r["trial_id"]]["y"] for r in failed], marker="x", color="red", label="No scored output")
        axes[0, 1].legend(fontsize=8)
    axes[0, 1].set(title="Relocalization by test position", xlabel="World x (m)", ylabel="World y (m)")
    axes[0, 1].axis("equal")
    for axis, key, title, threshold in [(axes[1, 0], "position_error_m", "3D position error (m)", .1),
                                        (axes[1, 1], "orientation_error_deg", "SO(3) orientation error (deg)", 2)]:
        values = np.sort([float(r[key]) for r in valid])
        if len(values):
            axis.step(values, np.arange(1, len(values) + 1) / len(values), where="post")
        axis.axvline(threshold, color="red", linestyle="--", label="Acceptance threshold")
        axis.set(title="Accepted outputs: empirical CDF", xlabel=title, ylabel="Fraction of scored outputs", ylim=(0, 1.03))
        axis.legend(fontsize=8)
    for axis in axes.flat:
        axis.grid(alpha=.2)
    fig.savefig(a.batch / "evaluation.png", dpi=180)
    fig.savefig(a.batch / "evaluation.svg")
    plt.close(fig)
    drift_fig, drift_axes = plt.subplots(1, 3, figsize=(15, 4.4), layout="constrained")
    elapsed = np.array([r["stamp"] for r in errors]) - errors[0]["stamp"]
    signed = estimate[:, :3, 3] - truth[:, :3, 3]
    for i, name in enumerate(["x", "y", "z"]):
        drift_axes[0].plot(elapsed, signed[:, i], label=name, linewidth=1)
    drift_axes[0].set(xlabel="Elapsed simulation time (s)", ylabel="Estimated - truth (m)", title="Mapping position error components")
    drift_axes[0].legend()
    drift_axes[1].plot(elapsed, [r["orientation_so3_deg"] for r in errors], label="SO(3)", linewidth=1)
    drift_axes[1].plot(elapsed, [abs(r["yaw_error_deg"]) for r in errors], label="Absolute yaw", linewidth=1)
    drift_axes[1].set(xlabel="Elapsed simulation time (s)", ylabel="Error (deg)", title="Mapping orientation error")
    drift_axes[1].legend()
    section = all_points[::max(1, len(all_points) // 18000)]
    drift_axes[2].scatter(section[:, 0], section[:, 2], s=.3, alpha=.4, rasterized=True)
    drift_axes[2].plot(truth[:, 0, 3], truth[:, 2, 3], linewidth=1, color="red", label="True base trajectory")
    drift_axes[2].set(xlabel="World x (m)", ylabel="World z (m)", title="Entire frozen map: X-Z projection")
    drift_axes[2].legend(fontsize=8)
    for axis in drift_axes:
        axis.grid(alpha=.2)
    drift_fig.savefig(a.batch / "mapping_drift.png", dpi=180)
    drift_fig.savefig(a.batch / "mapping_drift.svg")
    plt.close(drift_fig)

    metrics = [("三维位置误差 / m", "translation_3d_m"), ("平面位置误差 / m", "translation_xy_m"),
               ("完整姿态 SO(3) / °", "orientation_so3_deg"), ("绝对航向误差 / °", "absolute_yaw_error_deg")]
    def number(value):
        return "—" if value is None else f"{value:.6f}"
    table = ""
    for title, key in metrics:
        row = summary[key]
        table += "<tr><th>" + title + "</th>" + "".join("<td>" + number(row[k]) + "</td>" for k in ["mean", "rmse", "median", "p95", "max"]) + "</tr>"
    failure_reasons = failure_reasons_for_profile(trials, profile)
    failures = "".join(f"<li>{html.escape(reason)}：{count}</li>" for reason, count in failure_reasons.items()) or "<li>无</li>"
    group_rows = ""
    for yaw, group in summary["groups_by_initial_yaw"].items():
        position, orientation = group["translation_3d_m"], group["orientation_so3_deg"]
        group_rows += f"<tr><th>{np.degrees(float(yaw)):.0f}°</th><td>{group['strict_correct']}/{group['completed']}</td><td>{number(position['mean'])}</td><td>{number(position['p95'])}</td><td>{number(orientation['mean'])}</td><td>{number(orientation['p95'])}</td></tr>"
    strata = altitude_strata(frozen_trials, trials, profile)
    altitude_rows = "".join(
        f"<tr><th>{html.escape(height)} m</th><td>{group['completed']}/{group['planned']}</td>"
        f"<td>{group['strict']}/{group['planned']}</td><td>{group['profile']}/{group['planned']}</td>"
        f"<td>{group['fast']}/{group['planned']}</td></tr>"
        for height, group in sorted(strata.items(), key=lambda item: (item[0] == "未知", item[0])))
    image = base64.b64encode((a.batch / "evaluation.png").read_bytes()).decode()
    drift_image = base64.b64encode((a.batch / "mapping_drift.png").read_bytes()).decode()
    paired_note = ""
    paired_dir = a.batch.parent / "paired_maps"
    if (paired_dir / "summary.json").exists():
        paired = json.loads((paired_dir / "summary.json").read_text())
        paired_rows = [json.loads(p.read_text()) for p in sorted(paired_dir.glob("p*_yaw[0-3].json"))]
        shared = [r for r in paired_rows if "exception" not in r and all(r.get(k, {}).get("prepared_ready") for k in ["sensor_map", "ideal_map"])]
        compare_rows = ""
        for name, label in [("sensor_map", "车辆建图地图"), ("ideal_map", "理想地图")]:
            group = paired["groups"][name]
            mean = np.mean([r[name]["reference_score"]["translation_3d_m"] for r in shared]) if shared else None
            compare_rows += (f"<tr><th>{label}</th><td>{group['evaluated']}</td><td>{group['prepared_ready']}</td>"
                             f"<td>{group['reference_strict_correct_when_prepared']}</td><td>{group['full_confirmation_success']}</td>"
                             f"<td>{number(mean)}</td></tr>")
        examples = [r for r in paired_rows if "exception" not in r and all("current_base_score" in r.get(k, {}) for k in ["sensor_map", "ideal_map"])][:2]
        examples_html = "".join(f"<li>{r['trial_id']}：位置误差 {r['sensor_map']['current_base_score']['translation_3d_m']:.6f} → "
                                f"{r['ideal_map']['current_base_score']['translation_3d_m']:.6f} m；完整姿态 "
                                f"{r['sensor_map']['current_base_score']['orientation_so3_deg']:.6f} → "
                                f"{r['ideal_map']['current_base_score']['orientation_so3_deg']:.6f}°。</li>" for r in examples)
        paired_note = (f"<h2>只更换地图的离线对照</h2><p>已完成 {paired['completed']}/{paired['planned']} 组。"
                       f"其中 {paired.get('comparison_exceptions', 0)} 组对照运行异常，未进入有效对照统计。"
                       "查询云、可用的独立确认云、参数和重定位核心库完全相同，估计器不读取真值；只替换地图。"
                       "下表严格评分针对查询参考坐标系。原在线流程在准备阶段拒绝的组没有停车确认云，"
                       "这部分只比较准备阶段，不能当作完整定位成功。</p>"
                       "<table><tr><th>地图</th><th>已评估</th><th>准备通过</th><th>准备通过且参考位姿严格正确</th>"
                       "<th>完整独立确认成功</th><th>双方准备通过的同组位置均值 m</th></tr>"
                       f"{compare_rows}</table><p>最后一列限定为双方均通过准备的相同 {len(shared)} 组，保证统计样本一致。"
                       f"另有 {paired['ideal_ready_sensor_rejected']} 组在建图地图上准备失败、换理想地图后准备通过。"
                       "完整确认样本的底盘位姿用原始局部轨迹合成，并与在线输出核对。</p>"
                       f"<p>按测试编号最早的两组完整确认对照：</p><ul>{examples_html}</ul>"
                       "<p>逐组双地图输出、冻结输入哈希、回放一致性及对照统计保存在同级 paired_maps/ 中。</p>")
    report = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>传感器建图全局重定位评估</title>
<style>body{{max-width:1180px;margin:40px auto;padding:0 24px;font:16px/1.65 system-ui;color:#182230}}table{{border-collapse:collapse;width:100%}}td,th{{padding:9px;border:1px solid #d5dce5;text-align:right}}th:first-child{{text-align:left}}img{{width:100%}}code{{overflow-wrap:anywhere}}</style>
<h1>{method_name} 建图地图上的全局重定位测试</h1>
{termination_note}
{acceptance_panel}
<p>已完成 {summary['completed']} / {summary['planned']} 组；算法接受 {summary['accepted']} 组；有同步真值评分 {summary['valid_scored']} 组；严格正确 {summary['strict_correct']} 组。
严格标准：三维位置误差 ≤ 0.10 m 且完整姿态 SO(3) 误差 ≤ 2°。</p>
<p>sensor_map 标准：三维位置误差 ≤ 0.15 m 且完整姿态 SO(3) 误差 ≤ 3°。最终验收要求选定 profile 下精度正确且 trigger-to-pose ≤ 1 s 的全部计划组均通过，并通过冻结输入与 holdout freshness 审计。</p>
<p>以下误差统计包含全部有有效真值的算法接受结果，包括误定位；拒绝、超时、缺失真值不会记成零误差。验收成功率以全部计划组为分母。</p>
<table><tr><th>指标</th><th>均值</th><th>RMSE</th><th>中位数</th><th>P95</th><th>最大值</th></tr>{table}</table>
<h2>按初始朝向分组</h2><table><tr><th>朝向</th><th>严格正确/总数</th><th>位置均值 m</th><th>位置 P95 m</th><th>姿态均值 °</th><th>姿态 P95 °</th></tr>{group_rows}</table>
<h2>按请求高度分层</h2><p>按请求底盘高度四舍五入至 0.01 m 分组；分母包含该层全部计划试次，缺失、拒绝和未完成项不记为成功。</p>
<table><tr><th>请求高度</th><th>完成/计划</th><th>严格正确/计划</th><th>{html.escape(profile)} 精度正确/计划</th><th>精度且 ≤1 s / 计划</th></tr>{altitude_rows}</table>
<img src="data:image/png;base64,{image}" alt="建图轨迹、测试位置和误差分布">
<h2>建图与坐标约定</h2>
<p>使用仓库 {method_name} 和原有点云转换器，对连续行驶采集的原始 LiDAR/IMU 序列重新建图，共融合 {manifest['fused_scans']} 帧。
建图 use_imu_as_input={params['use_imu_as_input']}，gyr_cov_input={params['mapping']['gyr_cov_input']}、acc_cov_input={params['mapping']['acc_cov_input']}。全局重定位配置不作调整。
地图 {manifest['points']} 点。{control_note}二维已有栅格仅规划采集路线。
采集后仅按第一个共同时间戳固定一次地图坐标，未拟合整段轨迹、未按每帧真值融合或校正漂移。</p>
{source_note}
{method_note}
<p>建图轨迹三维误差均值 {mapping['translation_3d_m']['mean']:.6f} m，P95 {mapping['translation_3d_m']['p95']:.6f} m；
姿态均值 {mapping['orientation_so3_deg']['mean']:.6f}°，P95 {mapping['orientation_so3_deg']['p95']:.6f}°。</p>
<img src="data:image/png;base64,{drift_image}" alt="建图 XYZ 分量误差、完整姿态误差和完整先验地图 XZ 投影">
<p>重定位真值在输出位姿时间戳处线性插值位置、SLERP 插值四元数；插值间隔最大 0.25 秒，不外推。</p>
<p>边界：Gazebo 原始扫描为瞬时采样，沿用的建图转换器分配合成逐点时间；LiDAR 标称距离噪声 2 mm，IMU 未额外加噪。
这是传感器建图仿真，不等同于真实 MID360 逐点扫描或实车验证。详细诊断与录包开启，耗时包含记录开销。</p>
<h2>未通过原因</h2><ul>{failures}</ul>
{analysis_note}
{paired_note}
<p>逐组位姿及误差：results.csv；完整统计：summary.json；每组目录保存 result.json、原始 bag、参数审计、节点日志和配准诊断。
定位使用的 map.pcd SHA256：<code>{manifest['map_sha256']}</code>。</p></html>"""
    (a.batch / "report.html").write_text(report)
    print(json.dumps({"report": str(a.batch / "report.html"), "strict_correct": summary["strict_correct"], "completed": summary["completed"]}))


if __name__ == "__main__":
    main()
