"""根据工作区中的源模型生成 AS2-W 与 R1-A7-O6 统一 MJCF。"""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


from centaur_control.paths import PROJECT_ROOT as ROOT
URDF = ROOT / "as2w_model" / "as2w.urdf"
R1 = ROOT / "r1_a7_o6_model" / "r1_a7_o6_model" / "r1_a7_O6.xml"
OUTPUT = ROOT / "centaur_control" / "centaur_unified.xml"


def vec(node, path, default="0 0 0"):
    child = node.find(path)
    return child.get("xyz", default) if child is not None else default


def build():
    robot = ET.parse(URDF).getroot()
    upper = ET.parse(R1).getroot()
    model = ET.Element("mujoco", model="AS2W_R1A7_O6_unified")
    ET.SubElement(model, "compiler", angle="radian", autolimits="true", meshdir="../")
    ET.SubElement(model, "option", timestep="0.002", gravity="0 0 -9.81", integrator="implicitfast", cone="elliptic")
    ET.SubElement(model, "size", njmax="3000", nconmax="1000")

    defaults = deepcopy(upper.find("default"))
    for default in defaults.iter("joint"):
        if default.get("damping") is None:
            default.set("damping", "0.1")
    model.append(defaults)

    assets = ET.SubElement(model, "asset")
    source_dir = R1.parent
    for asset in upper.find("asset"):
        item = deepcopy(asset)
        if item.get("file"):
            item.set("file", (source_dir / item.get("file")).relative_to(ROOT).as_posix())
        assets.append(item)

    links = {link.get("name"): link for link in robot.findall("link")}
    joints_by_parent = {}
    for joint in robot.findall("joint"):
        joints_by_parent.setdefault(joint.find("parent").get("link"), []).append(joint)

    for link_name, link in links.items():
        item = ET.SubElement(assets, "mesh", name=f"as2w_{link_name}",
                             file=(ROOT / "as2w_model" / "meshes" / f"{link_name}.STL").relative_to(ROOT).as_posix())

    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", pos="0 0 2.5", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", friction="0.9 0.02 0.01", condim="3", rgba="0.55 0.58 0.60 1")
    base_link = links["base_link"]
    base = ET.SubElement(world, "body", name="base_link", pos="0 0 0.50")
    ET.SubElement(base, "freejoint", name="base_free")
    inertia = base_link.find("inertial")
    ET.SubElement(base, "inertial", pos=vec(inertia, "origin"), mass=inertia.find("mass").get("value"),
                  fullinertia=" ".join(inertia.find("inertia").get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
    ET.SubElement(base, "geom", name="base_visual", type="mesh", mesh="as2w_base_link", contype="0", conaffinity="0", group="1", rgba="0.79 0.82 0.93 1")
    ET.SubElement(base, "geom", name="base_collision", type="mesh", mesh="as2w_base_link", contype="1", conaffinity="1", group="3", friction="0.8 0.02 0.01")

    def add_link(parent_body, link_name, joint=None):
        link = links[link_name]
        attrs = {"name": link_name}
        if joint is not None:
            origin = joint.find("origin")
            if origin is not None:
                attrs["pos"] = origin.get("xyz", "0 0 0")
                attrs["euler"] = origin.get("rpy", "0 0 0")
        body = ET.SubElement(parent_body, "body", attrs)
        inertial = link.find("inertial")
        if inertial is not None:
            inert = ET.SubElement(body, "inertial", pos=vec(inertial, "origin"), mass=inertial.find("mass").get("value"),
                                  fullinertia=" ".join(inertial.find("inertia").get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
        if joint is not None and joint.get("type") != "fixed":
            limit = joint.find("limit")
            attrs = {"name": joint.get("name"), "type": "hinge", "axis": joint.find("axis").get("xyz"), "damping": "0.15", "armature": "0.01"}
            if limit is not None and limit.get("lower") is not None:
                attrs.update(limited="true", range=f"{limit.get('lower')} {limit.get('upper')}")
            else:
                attrs["limited"] = "false"
            ET.SubElement(body, "joint", attrs)
        visual = link.find("visual")
        mesh_pos = vec(visual, "origin") if visual is not None else "0 0 0"
        mesh_rpy = visual.find("origin").get("rpy", "0 0 0") if visual is not None and visual.find("origin") is not None else "0 0 0"
        if link_name.endswith("_foot"):
            wheel_y = "-0.0534" if link_name.startswith(("FR", "RR")) else "0.0534"
            ET.SubElement(body, "geom", name=f"{link_name}_visual", type="mesh", mesh=f"as2w_{link_name}", pos=mesh_pos, euler=mesh_rpy, contype="0", conaffinity="0", group="1", rgba="0.25 0.28 0.30 1")
            ET.SubElement(body, "geom", name=f"{link_name}_wheel", type="cylinder", size="0.086 0.026", pos=f"0 {wheel_y} 0", quat="0.7071068 0.7071068 0 0", friction="1.0 0.02 0.01", condim="4", group="3")
            ET.SubElement(body, "site", name=f"{link_name}_site", pos=f"0 {wheel_y} 0", size="0.008", rgba="1 0.2 0.1 1")
        elif visual is not None:
            ET.SubElement(body, "geom", name=f"{link_name}_visual", type="mesh", mesh=f"as2w_{link_name}", pos=mesh_pos, euler=mesh_rpy, contype="0", conaffinity="0", group="1", rgba="0.79 0.82 0.93 1")
            collision = link.find("collision")
            if collision is not None:
                origin = collision.find("origin")
                ET.SubElement(body, "geom", name=f"{link_name}_collision", type="mesh", mesh=f"as2w_{link_name}", pos=origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", euler=origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", contype="1", conaffinity="1", group="3")
        if link_name == "JT128":
            return body
        for child_joint in joints_by_parent.get(link_name, []):
            child_name = child_joint.find("child").get("link")
            add_link(body, child_name, child_joint)
        return body

    for joint in joints_by_parent["base_link"]:
        if joint.find("child").get("link") != "JT128":
            add_link(base, joint.find("child").get("link"), joint)
    add_link(base, "JT128", next(j for j in robot.findall("joint") if j.get("name") == "JT128"))
    pelvis = deepcopy(upper.find("worldbody").find("body"))
    pelvis.set("pos", "-0.015 0 0.1")
    pelvis.set("quat", "1 0 0 0")
    for node in pelvis.iter():
        if node.tag == "geom" and node.get("mesh"):
            node.set("contype", "0" if node.get("group") == "1" else node.get("contype", "1"))
    for hand in ("lh_hand_base_link", "rh_hand_base_link"):
        hand_body = pelvis.find(f".//body[@name='{hand}']")
        if hand_body is not None:
            ET.SubElement(hand_body, "site", name=f"{hand}_site", pos="0 0 0.15", size="0.012", rgba="0.2 0.8 0.2 1")
    base.append(pelvis)

    actuators = ET.SubElement(model, "actuator")
    for actuator in upper.find("actuator"):
        item = deepcopy(actuator)
        item.set("ctrllimited", "true")
        actuators.append(item)
    for name, limit, kp in (
        ("hip", 60, 300), ("thigh", 60, 300), ("calf", 90, 300), ("foot", 15, 400)
    ):
        for leg in ("FL", "FR", "RL", "RR"):
            joint_name = f"{leg}_{name}"
            joint = next(j for j in robot.findall("joint") if j.get("name") == joint_name)
            jlim = joint.find("limit")
            ctrlrange = f"{jlim.get('lower')} {jlim.get('upper')}" if jlim is not None and jlim.get("lower") is not None else "-1000000 1000000"
            kv = 20.0
            ET.SubElement(actuators, "position", name=f"{joint_name}_pd", joint=joint_name, kp=str(kp), kv=str(kv), ctrlrange=ctrlrange, forcerange=f"{-limit} {limit}")

    if upper.find("equality") is not None:
        model.append(deepcopy(upper.find("equality")))
    contact = deepcopy(upper.find("contact")) if upper.find("contact") is not None else ET.Element("contact")
    # 这些装配网格在源姿态下有轻微重叠，排除后避免虚假的高接触力。
    for body in ("FL_thigh", "FR_thigh", "left_shoulder_yaw_link", "right_shoulder_yaw_link"):
        ET.SubElement(contact, "exclude", body1="base_link", body2=body)
    model.append(contact)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(model, space="  ")
    ET.ElementTree(model).write(OUTPUT, encoding="unicode", xml_declaration=True)
    return OUTPUT


def main():
    print(build())

if __name__ == "__main__":
    main()
