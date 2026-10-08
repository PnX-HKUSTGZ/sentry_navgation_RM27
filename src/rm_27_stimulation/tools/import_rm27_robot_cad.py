#!/usr/bin/env python3
"""Build lightweight RM27 robot visual meshes from a SolidWorks STL export."""

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import struct
import tempfile
import zipfile

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = PACKAGE_DIR / "meshes" / "rm27_sentry_visual"
LIDAR_ORIGIN = np.array([0.0, 0.126, 0.130])
CAD_LIDAR_ROLL = np.deg2rad(30.0)
LIDAR_ROLL = np.deg2rad(-30.0)

# These are construction/reference bodies, not physical robot parts.
EXCLUDED_NAME_FRAGMENTS = (
    "地面",
    "图传范围",
    "线干涉",
    "雷达位置确认",
    "安装要求",
    "放置",
)


def find_default_archive():
    candidates = sorted(REPOSITORY_DIR.glob("*922*.zip"))
    if len(candidates) != 1:
        names = ", ".join(path.name for path in candidates) or "none"
        raise ValueError(
            "Expected exactly one *922*.zip archive in the repository root; "
            f"found: {names}. Pass --archive explicitly."
        )
    return candidates[0]


def read_binary_stl(archive, info):
    data = archive.read(info)
    if len(data) < 84:
        raise ValueError(f"STL is too short: {info.filename}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * triangle_count:
        raise ValueError(f"Only binary STL is supported: {info.filename}")
    records = np.ndarray(
        (triangle_count, 12),
        dtype="<f4",
        buffer=data,
        offset=84,
        strides=(50, 4),
    )
    return records[:, 3:12].reshape(-1, 3).copy()


def classify_part(name):
    lowered = name.lower()
    if "全向轮" in name:
        return "wheels"
    if "装甲" in name:
        return "armor"
    if "mid-360" in lowered or "mid360" in lowered or "雷达" in name:
        return "sensors"
    return "body"


def simplify_part(vertices, voxel_mm):
    quantized = np.rint(vertices / voxel_mm).astype(np.int32)
    unique_vertices, inverse = np.unique(quantized, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    valid = (
        (faces[:, 0] != faces[:, 1])
        & (faces[:, 1] != faces[:, 2])
        & (faces[:, 0] != faces[:, 2])
    )
    faces = faces[valid]
    if not len(faces):
        return np.empty((0, 3, 3), dtype=np.float32)

    # Remove coincident faces introduced by clustering while retaining the
    # original winding of the first face.
    face_keys = np.sort(faces, axis=1)
    _, first = np.unique(face_keys, axis=0, return_index=True)
    faces = faces[np.sort(first)]
    return (unique_vertices[faces] * voxel_mm).astype(np.float32)


def cad_to_ros(triangles, center_x_mm, center_z_mm, axle_y_mm):
    result = np.empty_like(triangles, dtype=np.float32)
    result[:, :, 0] = (triangles[:, :, 2] - center_z_mm) * 0.001
    result[:, :, 1] = (triangles[:, :, 0] - center_x_mm) * 0.001
    result[:, :, 2] = (triangles[:, :, 1] - axle_y_mm) * 0.001
    return result


def yaw_rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def roll_rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def discover_gimbal_alignment(archive, mesh_infos, alignment):
    references = {}
    suffixes = {
        "radar_left": "小yaw-板件-3mm雷达支撑板-1.STL",
        "radar_right": "小yaw-板件-3mm雷达支撑板-4.STL",
        "small_yaw_pivot": "大yaw-911-1 标准件-DM4310关节电机-1.STL",
        "camera_plate": "小yaw-板件-2mm相机固定侧板-1.STL",
    }
    for info in mesh_infos:
        for key, suffix in suffixes.items():
            if info.filename.endswith(suffix):
                vertices = read_binary_stl(archive, info).reshape(-1, 3, 3)
                references[key] = cad_to_ros(
                    vertices, alignment["cad_center_x_mm"],
                    alignment["cad_center_z_mm"], alignment["cad_axle_y_mm"],
                ).reshape(-1, 3).astype(np.float64)
    missing = set(suffixes) - set(references)
    if missing:
        raise ValueError(f"Cannot align gimbals; missing CAD references: {sorted(missing)}")

    def center(key):
        points = references[key]
        return 0.5 * (points.min(axis=0) + points.max(axis=0))

    radar_axis = center("radar_left") - center("radar_right")
    radar_yaw = float(np.arctan2(radar_axis[1], radar_axis[0]))
    big_correction = float(np.pi - radar_yaw)
    _, axes = np.linalg.eigh(np.cov(references["camera_plate"][:, :2].T))
    camera_axis = axes[:, -1]
    pivot = center("small_yaw_pivot")
    if camera_axis.dot((center("camera_plate") - pivot)[:2]) < 0.0:
        camera_axis = -camera_axis
    small_correction = -float(np.arctan2(camera_axis[1], camera_axis[0]))
    return {
        "big_yaw_correction_deg": float(np.rad2deg(big_correction)),
        "small_yaw_correction_deg": float(np.rad2deg(small_correction)),
        "small_yaw_pivot_m": pivot.tolist(),
        "source_lidar_yaw_rad": radar_yaw,
        "source_lidar_roll_rad": float(CAD_LIDAR_ROLL),
        "source_lidar_origin_m": [0.065, -0.119, 0.125],
        "target_lidar_origin_m": LIDAR_ORIGIN.tolist(),
        "target_lidar_rpy_rad": [float(LIDAR_ROLL), 0.0, 0.0],
    }


def align_gimbal(triangles, name, alignment):
    big_rotation = yaw_rotation(np.deg2rad(alignment["big_yaw_correction_deg"]))
    if " - 大yaw-911-" in name and "雷达" in name:
        # Rotate the sensor and its brackets from the CAD attitude to the
        # requested mount; source and target roll have opposite signs.
        source_rotation = (
            yaw_rotation(alignment["source_lidar_yaw_rad"])
            @ roll_rotation(alignment["source_lidar_roll_rad"])
        )
        rotation = roll_rotation(LIDAR_ROLL) @ source_rotation.T
        return ((triangles - alignment["source_lidar_origin_m"]) @ rotation.T + LIDAR_ORIGIN).astype(np.float32)
    if " - 大yaw-911-" in name:
        return (triangles @ big_rotation.T).astype(np.float32)
    if " - 小YAW-922-" in name:
        pivot = np.asarray(alignment["small_yaw_pivot_m"])
        rotation = yaw_rotation(np.deg2rad(alignment["small_yaw_correction_deg"]))
        return ((triangles - pivot) @ rotation.T + big_rotation @ pivot).astype(np.float32)
    return triangles


def write_binary_stl(path, triangles, label):
    edges_a = triangles[:, 1] - triangles[:, 0]
    edges_b = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edges_a, edges_b)
    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 1.0e-12
    normals[nonzero] /= lengths[nonzero, None]
    normals[~nonzero] = 0.0

    dtype = np.dtype(
        [
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ]
    )
    records = np.zeros(len(triangles), dtype=dtype)
    records["normal"] = normals
    records["vertices"] = triangles
    header = label.encode("ascii", errors="replace")[:80].ljust(80, b" ")
    with path.open("wb") as output:
        output.write(header)
        output.write(struct.pack("<I", len(triangles)))
        records.tofile(output)


def discover_alignment(archive, mesh_infos):
    wheel_centers = []
    wheel_y_centers = []
    physical_y_min = float("inf")
    for info in mesh_infos:
        name = PurePosixPath(info.filename).name
        if any(fragment in name for fragment in EXCLUDED_NAME_FRAGMENTS):
            continue
        vertices = read_binary_stl(archive, info)
        physical_y_min = min(physical_y_min, float(vertices[:, 1].min()))
        if "轮盘零件1-1" in name:
            lower = vertices.min(axis=0)
            upper = vertices.max(axis=0)
            center = 0.5 * (lower + upper)
            wheel_centers.append(center[[0, 2]])
            wheel_y_centers.append(float(center[1]))
    if len(wheel_centers) != 4:
        raise ValueError(
            f"Expected four wheel-disc meshes, found {len(wheel_centers)}"
        )
    centers = np.asarray(wheel_centers)
    return {
        "cad_center_x_mm": float(centers[:, 0].mean()),
        "cad_center_z_mm": float(centers[:, 1].mean()),
        "cad_axle_y_mm": float(np.mean(wheel_y_centers)),
        "physical_y_min_mm": physical_y_min,
    }


def import_meshes(archive_path, output_path, voxel_mm, force):
    if output_path.exists() and not force:
        raise ValueError(f"Output exists: {output_path}; pass --force to replace it")

    grouped = {name: [] for name in ("body", "wheels", "armor", "sensors")}
    source_triangles = {name: 0 for name in grouped}
    source_files = {name: 0 for name in grouped}
    excluded = []

    with zipfile.ZipFile(archive_path) as archive:
        mesh_infos = [
            info
            for info in archive.infolist()
            if not info.is_dir() and info.filename.lower().endswith(".stl")
        ]
        alignment = discover_alignment(archive, mesh_infos)
        gimbal_alignment = discover_gimbal_alignment(archive, mesh_infos, alignment)
        for info in mesh_infos:
            name = PurePosixPath(info.filename).name
            if any(fragment in name for fragment in EXCLUDED_NAME_FRAGMENTS):
                excluded.append(name)
                continue
            vertices = read_binary_stl(archive, info)
            group = classify_part(name)
            source_triangles[group] += len(vertices) // 3
            source_files[group] += 1
            triangles = simplify_part(vertices, voxel_mm)
            if len(triangles):
                grouped[group].append(
                    align_gimbal(cad_to_ros(
                        triangles,
                        alignment["cad_center_x_mm"],
                        alignment["cad_center_z_mm"],
                        alignment["cad_axle_y_mm"],
                    ), name, gimbal_alignment)
                )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".rm27-robot-", dir=output_path.parent))
    metadata = {
        "source_archive": archive_path.name,
        "source_sha256": hashlib.sha256(archive_path.read_bytes()).hexdigest(),
        "voxel_mm": voxel_mm,
        "coordinate_mapping": "ros_x=cad_z, ros_y=cad_x, ros_z=cad_y",
        "alignment": alignment,
        "gimbal_alignment": gimbal_alignment,
        "excluded_files": excluded,
        "groups": {},
    }
    all_lower = np.full(3, np.inf)
    all_upper = np.full(3, -np.inf)
    try:
        for group, chunks in grouped.items():
            if not chunks:
                raise ValueError(f"No physical triangles were classified as {group}")
            triangles = np.concatenate(chunks)
            lower = triangles.min(axis=(0, 1))
            upper = triangles.max(axis=(0, 1))
            all_lower = np.minimum(all_lower, lower)
            all_upper = np.maximum(all_upper, upper)
            write_binary_stl(stage / f"{group}.stl", triangles, f"RM27 {group}")
            metadata["groups"][group] = {
                "source_files": source_files[group],
                "source_triangles": source_triangles[group],
                "output_triangles": int(len(triangles)),
                "bounds_m": {"min": lower.tolist(), "max": upper.tolist()},
            }
        metadata["combined_bounds_m"] = {
            "min": all_lower.tolist(),
            "max": all_upper.tolist(),
            "size": (all_upper - all_lower).tolist(),
        }
        (stage / "import_metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if output_path.exists():
            shutil.rmtree(output_path)
        os.replace(stage, output_path)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--voxel-mm",
        type=float,
        default=2.0,
        help="Vertex-clustering cell size in CAD millimetres (default: 2.0)",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if not np.isfinite(args.voxel_mm) or args.voxel_mm <= 0.0:
        parser.error("--voxel-mm must be finite and positive")
    try:
        archive = (args.archive or find_default_archive()).resolve()
        if not archive.is_file():
            parser.error(f"Archive does not exist: {archive}")
        metadata = import_meshes(
            archive, args.output.resolve(), args.voxel_mm, args.force
        )
    except (ValueError, zipfile.BadZipFile) as error:
        parser.error(str(error))

    output_triangles = sum(
        group["output_triangles"] for group in metadata["groups"].values()
    )
    source_triangles = sum(
        group["source_triangles"] for group in metadata["groups"].values()
    )
    size = metadata["combined_bounds_m"]["size"]
    print(
        f"Imported {source_triangles} source triangles as {output_triangles} "
        f"triangles ({100.0 * output_triangles / source_triangles:.1f}%)."
    )
    print(f"Physical CAD bounds: {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m")
    print(f"Output: {args.output.resolve()}")


if __name__ == "__main__":
    main()
