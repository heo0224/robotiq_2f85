#!/usr/bin/env python3
"""Extract a standalone 2F-85 URDF from the polymetis panda+gripper model.

The source URDF describes the whole arm; for the gripper UI we only want the
robotiq subtree, re-rooted at robotiq_2f_85_base.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).parent
# Vendored copy of the polymetis panda+gripper model, so this folder is
# self-contained and does not break if fairo moves or changes.
SRC = HERE / "source_panda_robotiq_85.urdf"
OUT = HERE / "robotiq_2f85.urdf"
PREFIX = "robotiq_2f_85_"

root = ET.parse(SRC).getroot()
keep_links = {l.get("name") for l in root.findall("link") if l.get("name").startswith(PREFIX)}

new = ET.Element("robot", {"name": "robotiq_2f_85"})
for link in root.findall("link"):
    if link.get("name") in keep_links:
        new.append(link)
for joint in root.findall("joint"):
    parent = joint.find("parent").get("link")
    child = joint.find("child").get("link")
    # drop the joints that attach the gripper to the arm; keep the internal ones
    if parent in keep_links and child in keep_links:
        new.append(joint)

# Two fixes to the mesh references, so they resolve against this folder's own
# flat meshes/ directory:
#
# 1. The polymetis tree ships only .mtl material files under meshes/.../visual/,
#    not the .obj geometry they describe, so every visual mesh except pad.stl is
#    a dangling reference. The collision STLs are complete, so both visual and
#    collision geometry now point at the same vendored STL.
# 2. The scales must be replaced too, not just the filenames: the visual entries
#    carry scales meant for the missing .obj files (0.1, and 0.0001 for pad),
#    while these STLs are modelled in millimetres and so need 0.001 like the
#    collision entries that already reference them.
retargeted = 0
for mesh in new.iter("mesh"):
    stem = mesh.get("filename").rsplit("/", 1)[1].rsplit(".", 1)[0]
    mesh.set("filename", f"meshes/{stem}.stl")
    mesh.set("scale", "0.001 0.001 0.001")
    retargeted += 1

ET.indent(new, space="  ")
OUT.write_bytes(ET.tostring(new, encoding="utf-8", xml_declaration=True))

links = len(new.findall("link"))
joints = new.findall("joint")
actuated = [j.get("name") for j in joints
            if j.get("type") == "revolute" and j.find("mimic") is None]
print(f"wrote {OUT}")
print(f"  links={links}  joints={len(joints)}")
print(f"  actuated joint: {actuated}")
print(f"  mesh refs retargeted to local meshes/: {retargeted}")
missing = sorted({m.get("filename") for m in new.iter("mesh")
                  if not (HERE / m.get("filename")).exists()})
print("  missing meshes:", missing if missing else "none")
