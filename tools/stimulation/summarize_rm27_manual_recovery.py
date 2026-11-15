#!/usr/bin/env python3
"""汇总人工恢复录包：显示延迟、恢复时间、错误提示截止与未定位阶段速度。"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import statistics
import subprocess


# 保留每次请求的失败和取消，不把被新点击替换的请求计为恢复成功。
def summarize(path):
    data = json.loads((path / "results.json").read_text())
    events = []
    for i, click in enumerate(data["clicks"]):
        begin = click["wall"]
        end = (
            data["clicks"][i + 1]["wall"]
            if i + 1 < len(data["clicks"])
            else float("inf")
        )
        states = [s for s in data["states"] if begin <= s["wall"] < end]
        lost = next((s for s in states if not s["trusted"]), None)
        restored = next(
            (s for s in states if lost and s["wall"] > lost["wall"] and s["trusted"]),
            None,
        )
        ambiguous = next(
            (s for s in states if "ambiguous" in s["reason"].lower()), None
        )
        shown = next(
            (
                s
                for s in data.get("displayed", [])
                if begin <= s["wall"] < end
                and math.hypot(
                    s["x"] - click["position"][0], s["y"] - click["position"][1]
                )
                < 0.15
            ),
            None,
        )
        failed = next(
            (
                s
                for s in data.get("markers", [])
                if begin <= s["wall"] < end and any("FAILED" in t for t in s["text"])
            ),
            None,
        )
        event = dict(
            click=click,
            replaced=i + 1 < len(data["clicks"]) and restored is None,
            recovery_s=restored["wall"] - begin if restored else None,
            preview_s=shown["wall"] - begin if shown else None,
            ambiguity_s=ambiguous["wall"] - begin if ambiguous else None,
            failure_s=failed["wall"] - begin if failed else None,
        )
        events.append(event)
    resources = data.get("resources", [])
    report = dict(
        events=events,
        goals=data.get("goals", []),
        trusted_samples=sum(s["trusted"] for s in data["states"]),
        samples=len(data["states"]),
        cancelled_jobs=max(
            (int(v.get("map_cancelled_jobs", 0)) for v in data["diagnostics"]),
            default=0,
        ),
    )
    if resources:
        report["resources"] = dict(
            samples=len(resources),
            cpu_mean=statistics.mean(s["cpu"] for s in resources),
            rss_max_mib=max(s["rss"] for s in resources) / 2**20,
            pss_max_mib=max(s["pss"] for s in resources) / 2**20,
        )
    # 仅使用可信阶段显示的底盘位姿与独立 GT 同时间比较，避免把预览当定位精度。
    stamped = [s for s in data.get("displayed", []) if "stamp" in s]
    if stamped and data["truth"]:
        import numpy as np

        gt = np.asarray(data["truth"])
        walls = [s["wall"] for s in data["states"]]
        errors = []
        for point in stamped:
            idx = bisect.bisect_right(walls, point["wall"]) - 1
            if idx < 0 or not data["states"][idx]["trusted"]:
                continue
            stamp = point["stamp"]
            if stamp < gt[0, 0] or stamp > gt[-1, 0]:
                continue
            truth = np.array([np.interp(stamp, gt[:, 0], gt[:, j]) for j in [1, 2, 3]])
            errors.append(
                float(
                    np.linalg.norm(
                        np.array([point["x"], point["y"], point["z"]]) - truth
                    )
                )
            )
        if errors:
            report["trusted_base_position"] = dict(
                samples=len(errors),
                rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
                max_m=max(errors),
            )
    return report


# 录包检查仅统计未定位区间内的实际最终速度；容许状态与停车消息的异步交付时间。
def inspect_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path / "bag")),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    from tf2_ros import Buffer
    from rclpy.duration import Duration
    from rclpy.time import Time

    buffer = Buffer(cache_time=Duration(seconds=120.0))
    pose_stamps = set()
    truth = []
    trusted = False
    blocked_since = None
    count = nonzero = 0
    while reader.has_next():
        topic, raw, record = reader.read_next()
        if topic not in [
            "/rm27_backend/localization_status",
            "/cmd_vel",
            "/tf",
            "/tf_static",
            "/ground_truth/odometry",
        ]:
            continue
        message = deserialize_message(raw, types[topic])
        t = record / 1e9
        if topic in ["/tf", "/tf_static"]:
            for transform in message.transforms:
                if topic == "/tf_static":
                    buffer.set_transform_static(transform, "bag")
                else:
                    buffer.set_transform(transform, "bag")
            continue
        if topic == "/ground_truth/odometry":
            p = message.pose.pose.position
            truth.append(
                [
                    message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                    p.x,
                    p.y,
                    p.z,
                ]
            )
            continue
        if topic.endswith("localization_status"):
            if message.global_trusted:
                pose_stamps.add(
                    message.pose_stamp.sec * 1000000000 + message.pose_stamp.nanosec
                )
                trusted = True
                blocked_since = None
            else:
                if trusted or blocked_since is None:
                    blocked_since = t
                trusted = False
        elif not trusted and blocked_since is not None and t - blocked_since > 0.05:
            count += 1
            nonzero += (
                math.hypot(message.linear.x, message.linear.y) > 1e-4
                or abs(message.angular.z) > 1e-4
            )
    result = dict(untrusted_velocity_samples_after_50ms=count, nonzero=nonzero)
    # 从完整 TF 历史按已提交状态的源时刻查询底盘，供修改前后使用相同评分方法。
    import numpy as np

    gt = np.asarray(truth)
    errors = []
    missing = 0
    for stamp in sorted(pose_stamps):
        try:
            transform = buffer.lookup_transform(
                "map", "base_footprint", Time(nanoseconds=stamp)
            )
        except Exception:
            missing += 1
            continue
        t = stamp / 1e9
        if not len(gt) or t < gt[0, 0] or t > gt[-1, 0]:
            continue
        p = transform.transform.translation
        expected = np.array([np.interp(t, gt[:, 0], gt[:, i]) for i in [1, 2, 3]])
        errors.append(float(np.linalg.norm(np.array([p.x, p.y, p.z]) - expected)))
    if errors:
        result["trusted_base_position"] = dict(
            samples=len(errors),
            rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
            max_m=max(errors),
            missing_tf_queries=missing,
        )
    return result


# 保存配置产物及哈希，标明仿真和实际传感器输入来源。
def provenance(path):
    config_dir = path / "params"
    config_dir.mkdir(exist_ok=True)
    log = (path / "launch.log").read_text(errors="replace")
    for name in set(re.findall(r"--params-file ([^\s\x1b\']+)", log)):
        source = Path(name)
        if source.is_file():
            shutil.copy2(source, config_dir / source.name)
    sources = list(config_dir.iterdir()) + [
        Path("src/pnx-launch/launch_bringup/config/simulation/minco_params.yaml"),
        Path("src/pnx-launch/launch_bringup/pcd/simulation/RMUC2027.pcd"),
        Path(
            "src/pnx-launch/launch_bringup/pcd/simulation/RMUC2027.pcd.rm27/manifest.yaml"
        ),
    ]
    return dict(
        revision=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        dirty_tree=True,
        world="RMUC2027",
        localization="Point-LIO + rm27_backend prior_map",
        navigation="minco",
        gui=False,
        ground_truth="scoring only",
        sha256={
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sources
            if p.is_file()
        },
    )


# 每个实验独立写出汇总，不覆盖原始 results.json 和 bag。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    for path in args.runs:
        result = summarize(path)
        result["safety"] = inspect_bag(path)
        (path / "summary.json").write_text(json.dumps(result, indent=2))
        (path / "provenance.json").write_text(json.dumps(provenance(path), indent=2))
        print(path.name, json.dumps(result, ensure_ascii=False))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
