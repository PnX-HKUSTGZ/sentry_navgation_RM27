#!/usr/bin/env python3
"""Import the split RM27 bundle as separate visual and collision Gazebo models."""

import argparse
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = Path(__file__).resolve().parents[3]
DEFAULT_VISUAL_OUTPUT = PACKAGE_DIR / "meshes" / "RM27_battlefield_visual"
DEFAULT_COLLISION_OUTPUT = PACKAGE_DIR / "meshes" / "RM27_battlefield_collision"
MODEL_MEMBER = "rm27_gazebo_harmonic/models/rm27_battlefield/model.sdf"
VISUAL_PREFIX = "rm27_gazebo_harmonic/models/rm27_battlefield/meshes/visual/"
COLLISION_PREFIX = "rm27_gazebo_harmonic/models/rm27_battlefield/meshes/collision/"
SOURCE_VISUAL_URI = "model://rm27_battlefield/meshes/visual/"
SOURCE_COLLISION_URI = "model://rm27_battlefield/meshes/collision/"
TARGET_VISUAL_URI = "model://RM27_battlefield_visual/meshes/visual/"
TARGET_COLLISION_URI = "model://RM27_battlefield_collision/meshes/collision/"
# Separate convex lintels at the two 250 mm tunnel openings. Bullet's default
# triangle-mesh margin is 10 mm; convex hulls use 1 mm without lowering the car.
TUNNEL_LINTEL_MESHES = frozenset(
    f"collision_{index:03}.stl" for index in (169, 171, 174, 176)
)


def read_member(archive, member_name):
    with tarfile.open(archive, "r:xz") as bundle:
        member = bundle.getmember(member_name)
        stream = bundle.extractfile(member)
        if stream is None:
            raise ValueError(f"Archive member is not a file: {member_name}")
        return stream.read()


def extract_meshes(archives, destination, prefix, extension, expected_count):
    extracted = set()
    mesh_kind = Path(prefix.rstrip("/")).name
    mesh_dir = destination / "meshes" / mesh_kind
    mesh_dir.mkdir(parents=True)

    for archive in archives:
        with tarfile.open(archive, "r:xz") as bundle:
            for member in bundle.getmembers():
                if not member.isfile() or not member.name.startswith(prefix):
                    continue
                filename = Path(member.name).name
                if Path(filename).suffix.lower() != extension:
                    continue
                if filename in extracted:
                    raise ValueError(
                        f"Duplicate {mesh_kind} mesh in split archives: {filename}"
                    )
                stream = bundle.extractfile(member)
                if stream is None:
                    raise ValueError(f"Cannot read {mesh_kind} mesh: {member.name}")
                with (mesh_dir / filename).open("wb") as output:
                    shutil.copyfileobj(stream, output)
                extracted.add(filename)

    if len(extracted) != expected_count:
        raise ValueError(
            f"Expected {expected_count} {extension} {mesh_kind} meshes, "
            f"extracted {len(extracted)}"
        )
    return extracted


def remove_elements(model, element_name):
    for link in model.findall("link"):
        for element in list(link.findall(element_name)):
            link.remove(element)


def rewrite_mesh_uris(model, source_prefix, target_prefix, meshes):
    referenced = set()
    for uri in model.iter("uri"):
        value = (uri.text or "").strip()
        if not value.startswith(source_prefix):
            raise ValueError(f"Unexpected mesh URI in source model: {value}")
        filename = value.removeprefix(source_prefix)
        if filename not in meshes:
            raise ValueError(f"Missing extracted mesh: {filename}")
        referenced.add(filename)
        uri.text = target_prefix + filename

    if referenced != meshes:
        unused = ", ".join(sorted(meshes - referenced))
        raise ValueError(f"Extracted mesh files are not referenced by model.sdf: {unused}")


def write_model_config(destination, name, author, description):
    config = ET.Element("model")
    ET.SubElement(config, "name").text = name
    ET.SubElement(config, "version").text = "1.0"
    sdf = ET.SubElement(config, "sdf", version="1.9")
    sdf.text = "model.sdf"
    author_element = ET.SubElement(config, "author")
    ET.SubElement(author_element, "name").text = author
    ET.SubElement(config, "description").text = description
    ET.indent(config, space="  ")
    ET.ElementTree(config).write(
        destination / "model.config", encoding="utf-8", xml_declaration=True
    )


def write_model(root, destination):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(
        destination / "model.sdf", encoding="utf-8", xml_declaration=True
    )


def add_bullet_friction(model):
    updated = 0
    for collision in model.iter("collision"):
        friction = collision.find("./surface/friction")
        ode = friction.find("ode") if friction is not None else None
        mu = ode.find("mu") if ode is not None else None
        mu2 = ode.find("mu2") if ode is not None else None
        if friction is None or mu is None or mu2 is None:
            raise ValueError(
                f"Collision {collision.get('name', '<unnamed>')} has no ODE friction"
            )

        existing = friction.find("bullet")
        if existing is not None:
            friction.remove(existing)
        bullet = ET.SubElement(friction, "bullet")
        ET.SubElement(bullet, "friction").text = (mu.text or "").strip()
        ET.SubElement(bullet, "friction2").text = (mu2.text or "").strip()
        ET.SubElement(bullet, "rolling_friction").text = "0.0"
        updated += 1
    return updated


def configure_tunnel_lintel_collisions(model):
    updated = set()
    for mesh in model.findall("./link/collision/geometry/mesh"):
        filename = Path((mesh.findtext("uri") or "").strip()).name
        if filename in TUNNEL_LINTEL_MESHES:
            mesh.set("optimization", "convex_hull")
            updated.add(filename)
    if updated != TUNNEL_LINTEL_MESHES:
        raise ValueError(f"Missing tunnel lintels: {sorted(TUNNEL_LINTEL_MESHES - updated)}")


def build_visual_model(source_xml, destination, meshes):
    root = ET.fromstring(source_xml)
    model = root.find("model")
    if model is None:
        raise ValueError("Source model.sdf does not contain a model")
    model.set("name", "RM27_battlefield_visual")
    remove_elements(model, "collision")

    visuals = list(model.iter("visual"))
    collisions = list(model.iter("collision"))
    if len(visuals) != 185 or collisions:
        raise ValueError(
            f"Expected 185 visuals and no collisions, got {len(visuals)} and {len(collisions)}"
        )

    rewrite_mesh_uris(model, SOURCE_VISUAL_URI, TARGET_VISUAL_URI, meshes)
    write_model(root, destination)
    write_model_config(
        destination,
        "RM27 Battlefield Visual",
        "CosmosMount source; visual-only integration",
        "RM27 detailed visual model. Physics is provided by its collision-only peer.",
    )


def build_collision_model(source_xml, destination, meshes):
    root = ET.fromstring(source_xml)
    model = root.find("model")
    if model is None:
        raise ValueError("Source model.sdf does not contain a model")
    model.set("name", "RM27_battlefield_collision")
    remove_elements(model, "visual")

    visuals = list(model.iter("visual"))
    collisions = list(model.iter("collision"))
    if visuals or len(collisions) != 607:
        raise ValueError(
            f"Expected no visuals and 607 collisions, got {len(visuals)} and {len(collisions)}"
        )

    friction_count = add_bullet_friction(model)
    if friction_count != len(collisions):
        raise ValueError(
            f"Expected Bullet friction on {len(collisions)} collisions, got {friction_count}"
        )

    rewrite_mesh_uris(model, SOURCE_COLLISION_URI, TARGET_COLLISION_URI, meshes)
    configure_tunnel_lintel_collisions(model)
    write_model(root, destination)
    write_model_config(
        destination,
        "RM27 Battlefield Collision",
        "CosmosMount source; collision-only integration",
        "RM27 physics model with 607 convex collision proxies and no visuals.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--part1",
        type=Path,
        default=REPOSITORY_DIR / "RM27_Gazebo_Harmonic_part1.tar.xz",
    )
    parser.add_argument(
        "--part2",
        type=Path,
        default=REPOSITORY_DIR / "RM27_Gazebo_Harmonic_part2.tar.xz",
    )
    parser.add_argument(
        "--visual-output", type=Path, default=DEFAULT_VISUAL_OUTPUT
    )
    parser.add_argument(
        "--collision-output", type=Path, default=DEFAULT_COLLISION_OUTPUT
    )
    parser.add_argument(
        "--force", action="store_true", help="Replace existing imported models"
    )
    args = parser.parse_args()

    archives = [path.resolve() for path in (args.part1, args.part2)]
    for archive in archives:
        if not archive.is_file():
            parser.error(f"Archive does not exist: {archive}")

    visual_output = args.visual_output.resolve()
    collision_output = args.collision_output.resolve()
    outputs = (visual_output, collision_output)
    for output in outputs:
        if output.exists() and not args.force:
            parser.error(f"Output exists: {output}; pass --force to replace it")
    if visual_output.parent != collision_output.parent:
        parser.error("Visual and collision outputs must have the same parent directory")

    visual_output.parent.mkdir(parents=True, exist_ok=True)
    stage_root = Path(
        tempfile.mkdtemp(prefix=".rm27-assets-", dir=visual_output.parent)
    )
    visual_stage = stage_root / "visual"
    collision_stage = stage_root / "collision"
    try:
        visual_meshes = extract_meshes(
            archives, visual_stage, VISUAL_PREFIX, ".dae", 85
        )
        collision_meshes = extract_meshes(
            archives, collision_stage, COLLISION_PREFIX, ".stl", 548
        )
        source_xml = read_member(archives[0], MODEL_MEMBER)
        build_visual_model(source_xml, visual_stage, visual_meshes)
        build_collision_model(source_xml, collision_stage, collision_meshes)

        for output in outputs:
            if output.exists():
                shutil.rmtree(output)
        os.replace(visual_stage, visual_output)
        os.replace(collision_stage, collision_output)
    finally:
        if stage_root.exists():
            shutil.rmtree(stage_root)

    print(f"Imported {len(visual_meshes)} DAE meshes into {visual_output}")
    print(f"Imported {len(collision_meshes)} STL meshes into {collision_output}")
    print("Generated 185 visuals and 607 collision proxies in separate models")


if __name__ == "__main__":
    main()
