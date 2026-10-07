#!/usr/bin/env python3
"""Import the split RM27 bundle as a visual-only Gazebo model."""

import argparse
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = PACKAGE_DIR / "meshes" / "RM27_battlefield_visual"
MODEL_MEMBER = "rm27_gazebo_harmonic/models/rm27_battlefield/model.sdf"
VISUAL_PREFIX = "rm27_gazebo_harmonic/models/rm27_battlefield/meshes/visual/"
SOURCE_URI = "model://rm27_battlefield/meshes/visual/"
TARGET_URI = "model://RM27_battlefield_visual/meshes/visual/"


def read_member(archive, member_name):
    with tarfile.open(archive, "r:xz") as bundle:
        member = bundle.getmember(member_name)
        stream = bundle.extractfile(member)
        if stream is None:
            raise ValueError(f"Archive member is not a file: {member_name}")
        return stream.read()


def extract_visual_meshes(archives, destination):
    extracted = set()
    mesh_dir = destination / "meshes" / "visual"
    mesh_dir.mkdir(parents=True)

    for archive in archives:
        with tarfile.open(archive, "r:xz") as bundle:
            for member in bundle.getmembers():
                if not member.isfile() or not member.name.startswith(VISUAL_PREFIX):
                    continue
                filename = Path(member.name).name
                if not filename.endswith(".dae"):
                    continue
                if filename in extracted:
                    raise ValueError(f"Duplicate visual mesh in split archives: {filename}")
                stream = bundle.extractfile(member)
                if stream is None:
                    raise ValueError(f"Cannot read visual mesh: {member.name}")
                with (mesh_dir / filename).open("wb") as output:
                    shutil.copyfileobj(stream, output)
                extracted.add(filename)

    if len(extracted) != 85:
        raise ValueError(f"Expected 85 DAE meshes, extracted {len(extracted)}")
    return extracted


def build_visual_model(source_xml, destination, meshes):
    root = ET.fromstring(source_xml)
    model = root.find("model")
    if model is None:
        raise ValueError("Source model.sdf does not contain a model")
    model.set("name", "RM27_battlefield_visual")

    for link in model.findall("link"):
        for collision in list(link.findall("collision")):
            link.remove(collision)

    visuals = list(model.iter("visual"))
    collisions = list(model.iter("collision"))
    if len(visuals) != 185 or collisions:
        raise ValueError(
            f"Expected 185 visuals and no collisions, got {len(visuals)} and {len(collisions)}"
        )

    referenced = set()
    for uri in model.iter("uri"):
        value = (uri.text or "").strip()
        if not value.startswith(SOURCE_URI):
            raise ValueError(f"Unexpected mesh URI in source model: {value}")
        filename = value.removeprefix(SOURCE_URI)
        if filename not in meshes:
            raise ValueError(f"Missing extracted visual mesh: {filename}")
        referenced.add(filename)
        uri.text = TARGET_URI + filename

    if referenced != meshes:
        unused = ", ".join(sorted(meshes - referenced))
        raise ValueError(f"Extracted DAE files are not referenced by model.sdf: {unused}")

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(
        destination / "model.sdf", encoding="utf-8", xml_declaration=True
    )

    config = ET.Element("model")
    ET.SubElement(config, "name").text = "RM27 Battlefield Visual"
    ET.SubElement(config, "version").text = "1.0"
    sdf = ET.SubElement(config, "sdf", version="1.9")
    sdf.text = "model.sdf"
    author = ET.SubElement(config, "author")
    ET.SubElement(author, "name").text = "CosmosMount source; visual-only integration"
    ET.SubElement(config, "description").text = (
        "RM27 detailed visual model. Collision is intentionally provided by the host world."
    )
    ET.indent(config, space="  ")
    ET.ElementTree(config).write(
        destination / "model.config", encoding="utf-8", xml_declaration=True
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
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--force", action="store_true", help="Replace an existing imported model"
    )
    args = parser.parse_args()

    archives = [path.resolve() for path in (args.part1, args.part2)]
    for archive in archives:
        if not archive.is_file():
            parser.error(f"Archive does not exist: {archive}")

    output = args.output.resolve()
    if output.exists() and not args.force:
        parser.error(f"Output exists: {output}; pass --force to replace it")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".rm27-visual-", dir=output.parent))
    try:
        meshes = extract_visual_meshes(archives, stage)
        source_xml = read_member(archives[0], MODEL_MEMBER)
        build_visual_model(source_xml, stage, meshes)
        if output.exists():
            shutil.rmtree(output)
        os.replace(stage, output)
    finally:
        if stage.exists():
            shutil.rmtree(stage)

    print(f"Imported {len(meshes)} DAE meshes into {output}")
    print("Generated visual-only model: 185 visuals, 0 collisions")


if __name__ == "__main__":
    main()
