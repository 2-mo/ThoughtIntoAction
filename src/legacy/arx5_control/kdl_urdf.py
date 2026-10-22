"""Small URDF compatibility patch for Orocos KDL.

KDL ignores inertia on the root link and emits a warning for the vendor X5
URDF.  A massless fixed parent preserves the exact fixed-base kinematics and
inverse dynamics while allowing KDL to retain the existing ``base_link``
inertia as a non-root link.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from embodied.devices.robots.arx5.paths import RUNTIME_ROOT


def kdl_compatible_urdf_path(
    urdf_path: str | Path,
    base_link_name: str,
    *,
    cache_root: Path | None = None,
) -> str:
    """Return a cached fixed-parent URDF when the KDL root needs one.

    Failure is intentionally non-fatal: the original vendor model is known to
    work, albeit with KDL's warning.  The generated file is content-addressed,
    so package upgrades automatically receive a fresh compatibility copy.
    """
    source = Path(urdf_path).expanduser().resolve()
    try:
        source_bytes = source.read_bytes()
        root = ET.fromstring(source_bytes)
    except (OSError, ET.ParseError):
        return str(source)

    base_link = next(
        (
            link
            for link in root.findall("link")
            if link.get("name") == base_link_name
        ),
        None,
    )
    if base_link is None or base_link.find("inertial") is None:
        return str(source)

    child_links = {
        child.get("link")
        for child in root.findall("joint/child")
        if child.get("link")
    }
    # KDL only warns when this is the actual tree root. Do not modify URDFs
    # that already provide a fixed world/dummy link.
    if base_link_name in child_links:
        return str(source)

    link_names = {link.get("name") for link in root.findall("link")}
    dummy_name = "arx5_kdl_root"
    suffix = 2
    while dummy_name in link_names:
        dummy_name = f"arx5_kdl_root_{suffix}"
        suffix += 1
    joint_name = f"{dummy_name}_to_{base_link_name}"

    digest = hashlib.sha256(source_bytes).hexdigest()[:16]
    root_directory = (
        cache_root
        if cache_root is not None
        else RUNTIME_ROOT / "cache" / "kdl-urdf"
    )
    destination = root_directory / f"{source.stem}-{digest}-kdl.urdf"
    if destination.is_file():
        return str(destination)

    dummy_link = ET.Element("link", {"name": dummy_name})
    fixed_joint = ET.Element("joint", {"name": joint_name, "type": "fixed"})
    ET.SubElement(fixed_joint, "parent", {"link": dummy_name})
    ET.SubElement(fixed_joint, "child", {"link": base_link_name})
    ET.SubElement(fixed_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    # Insert before existing links so exporters and simple URDF readers see
    # the root immediately; joint order has no kinematic meaning.
    root.insert(0, dummy_link)
    root.insert(1, fixed_joint)

    try:
        root_directory.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=root_directory,
            prefix=f".{source.stem}-",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            ET.ElementTree(root).write(
                stream,
                encoding="utf-8",
                xml_declaration=True,
            )
        os.replace(temporary, destination)
    except OSError:
        try:
            temporary.unlink(missing_ok=True)
        except UnboundLocalError:
            pass
        return str(source)
    return str(destination)
