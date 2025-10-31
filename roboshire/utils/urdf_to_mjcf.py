"""
URDF to MJCF Converter

Converts URDF robot models to MuJoCo MJCF format for physics simulation.

Author: RoboShire Team
Phase: 8 (Documentation & MuJoCo Integration)
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Dict, List, Tuple
import logging
import numpy as np


class URDFToMJCFConverter:
    """
    Convert URDF files to MuJoCo MJCF format

    Handles conversion of:
    - Links → Bodies
    - Joints → Joints
    - Visual geometry → Geoms
    - Collision geometry → Geoms with collision group
    - Inertial properties → Mass and inertia
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.used_geom_names = set()  # Track used geometry names

    def convert(self, urdf_path: str, output_path: Optional[str] = None) -> str:
        """
        Convert URDF file to MJCF format

        Args:
            urdf_path: Path to input URDF file
            output_path: Path for output MJCF file (optional)

        Returns:
            Path to generated MJCF file
        """
        urdf_path = Path(urdf_path)

        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        # Reset geom name tracking for new conversion
        self.used_geom_names.clear()

        # Parse URDF
        self.logger.info(f"Parsing URDF: {urdf_path}")
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Create MJCF root
        mjcf_root = ET.Element("mujoco", model=root.get("name", "robot"))

        # Add compiler settings
        compiler = ET.SubElement(mjcf_root, "compiler")
        compiler.set("angle", "radian")
        compiler.set("meshdir", str(urdf_path.parent / "meshes"))
        compiler.set("autolimits", "true")

        # Add options
        option = ET.SubElement(mjcf_root, "option")
        option.set("timestep", "0.001")
        option.set("gravity", "0 0 -9.81")

        # Add default settings
        default = ET.SubElement(mjcf_root, "default")
        geom_default = ET.SubElement(default, "geom")
        geom_default.set("rgba", "0.8 0.6 0.4 1")
        geom_default.set("friction", "1 0.5 0.5")

        # Add assets (materials, meshes)
        asset = ET.SubElement(mjcf_root, "asset")
        self._add_assets(root, asset, urdf_path.parent)

        # Add worldbody
        worldbody = ET.SubElement(mjcf_root, "worldbody")

        # Add lighting
        light = ET.SubElement(worldbody, "light")
        light.set("diffuse", "0.5 0.5 0.5")
        light.set("pos", "0 0 3")
        light.set("dir", "0 0 -1")

        # Add ground plane
        floor = ET.SubElement(worldbody, "geom")
        floor.set("name", "floor")
        floor.set("type", "plane")
        floor.set("size", "0 0 1")
        floor.set("rgba", "0.9 0.9 0.9 1")

        # Convert robot structure
        self._convert_robot(root, worldbody, urdf_path.parent)

        # Add actuators
        actuator = ET.SubElement(mjcf_root, "actuator")
        self._add_actuators(root, actuator)

        # Add sensors (optional)
        sensor = ET.SubElement(mjcf_root, "sensor")
        self._add_sensors(root, sensor)

        # Determine output path
        if output_path is None:
            output_path = urdf_path.parent / f"{urdf_path.stem}.xml"
        else:
            output_path = Path(output_path)

        # Write MJCF file
        self.logger.info(f"Writing MJCF: {output_path}")
        tree_out = ET.ElementTree(mjcf_root)
        ET.indent(tree_out, space="  ")
        tree_out.write(output_path, encoding="utf-8", xml_declaration=True)

        self.logger.info(f"Conversion complete: {output_path}")
        return str(output_path)

    def _add_assets(self, urdf_root: ET.Element, asset: ET.Element, base_path: Path):
        """Add materials and mesh assets"""
        # Add materials (avoid duplicates)
        added_materials = set()
        for material in urdf_root.findall(".//material"):
            mat_name = material.get("name")
            if mat_name and mat_name not in added_materials:
                mat_elem = ET.SubElement(asset, "material")
                mat_elem.set("name", mat_name)

                # Get color from URDF
                color_elem = material.find("color")
                if color_elem is not None:
                    rgba = color_elem.get("rgba", "0.8 0.8 0.8 1")
                    mat_elem.set("rgba", rgba)

                added_materials.add(mat_name)

        # Add meshes
        for mesh in urdf_root.findall(".//mesh"):
            filename = mesh.get("filename", "")
            if filename:
                # Handle package:// URIs
                if filename.startswith("package://"):
                    filename = filename.replace("package://", "")

                mesh_name = Path(filename).stem
                mesh_elem = ET.SubElement(asset, "mesh")
                mesh_elem.set("name", mesh_name)
                mesh_elem.set("file", filename)

                # Handle scale
                scale = mesh.get("scale")
                if scale:
                    mesh_elem.set("scale", scale)

    def _convert_robot(self, urdf_root: ET.Element, worldbody: ET.Element, base_path: Path):
        """Convert robot structure from URDF to MJCF"""
        # Find root link (link with no parent joint)
        all_links = {link.get("name") for link in urdf_root.findall("link")}
        child_links = {joint.find("child").get("link") for joint in urdf_root.findall("joint")}
        root_links = all_links - child_links

        if not root_links:
            self.logger.warning("No root link found, using first link")
            root_link = urdf_root.find("link")
        else:
            root_link_name = list(root_links)[0]
            root_link = urdf_root.find(f".//link[@name='{root_link_name}']")

        if root_link is not None:
            # Create root body with freejoint for floating base
            body = ET.SubElement(worldbody, "body")
            body.set("name", root_link.get("name"))
            body.set("pos", "0 0 0.5")  # Lift robot off ground

            # Add freejoint for floating base
            freejoint = ET.SubElement(body, "freejoint")
            freejoint.set("name", f"{root_link.get('name')}_freejoint")

            # Convert root link
            self._convert_link(root_link, body, base_path)

            # Recursively convert children
            self._convert_children(urdf_root, root_link.get("name"), body, base_path)

    def _convert_link(self, link: ET.Element, parent_body: ET.Element, base_path: Path):
        """Convert a single URDF link to MJCF body"""
        link_name = link.get("name")

        # Add inertial properties
        inertial = link.find("inertial")
        if inertial is not None:
            self._add_inertial(inertial, parent_body)

        # Add visual geometry
        for visual in link.findall("visual"):
            self._add_visual_geom(visual, parent_body, link_name, base_path)

        # Add collision geometry
        for collision in link.findall("collision"):
            self._add_collision_geom(collision, parent_body, link_name, base_path)

    def _add_inertial(self, inertial: ET.Element, body: ET.Element):
        """Add inertial properties to body"""
        # Get mass
        mass_elem = inertial.find("mass")
        if mass_elem is not None:
            mass_val = mass_elem.get("value", "1.0")
            inertial_elem = ET.SubElement(body, "inertial")
            inertial_elem.set("mass", mass_val)

            # Get center of mass (MJCF requires 'pos' attribute)
            origin = inertial.find("origin")
            if origin is not None:
                xyz = origin.get("xyz", "0 0 0")
                inertial_elem.set("pos", xyz)

                rpy = origin.get("rpy", "0 0 0")
                if rpy != "0 0 0":
                    # Convert RPY to quaternion for MuJoCo
                    quat = self._rpy_to_quat(rpy)
                    inertial_elem.set("quat", quat)
            else:
                # No origin specified, use default (0, 0, 0)
                inertial_elem.set("pos", "0 0 0")

            # Get inertia
            inertia_elem = inertial.find("inertia")
            if inertia_elem is not None:
                ixx = inertia_elem.get("ixx", "0.001")
                iyy = inertia_elem.get("iyy", "0.001")
                izz = inertia_elem.get("izz", "0.001")
                ixy = inertia_elem.get("ixy", "0")
                ixz = inertia_elem.get("ixz", "0")
                iyz = inertia_elem.get("iyz", "0")

                # MuJoCo uses full inertia matrix
                inertial_elem.set("fullinertia", f"{ixx} {iyy} {izz} {ixy} {ixz} {iyz}")

    def _ensure_unique_geom_name(self, base_name: str) -> str:
        """Ensure geom name is unique by appending index if needed"""
        geom_name = base_name
        counter = 1
        while geom_name in self.used_geom_names:
            geom_name = f"{base_name}_{counter}"
            counter += 1
        self.used_geom_names.add(geom_name)
        return geom_name

    def _add_visual_geom(self, visual: ET.Element, body: ET.Element, link_name: str, base_path: Path):
        """Add visual geometry"""
        geometry = visual.find("geometry")
        if geometry is None:
            return

        geom = ET.SubElement(body, "geom")
        base_geom_name = visual.get("name", f"{link_name}_visual")
        geom_name = self._ensure_unique_geom_name(base_geom_name)
        geom.set("name", geom_name)
        geom.set("type", "mesh")  # Will be overridden if primitive
        geom.set("group", "0")  # Visual group

        # Get origin
        origin = visual.find("origin")
        if origin is not None:
            xyz = origin.get("xyz", "0 0 0")
            geom.set("pos", xyz)

            rpy = origin.get("rpy", "0 0 0")
            if rpy != "0 0 0":
                quat = self._rpy_to_quat(rpy)
                geom.set("quat", quat)

        # Get geometry type
        self._add_geometry(geometry, geom)

        # Get material
        material = visual.find("material")
        if material is not None:
            mat_name = material.get("name")
            if mat_name:
                geom.set("material", mat_name)
            else:
                # Inline color
                color_elem = material.find("color")
                if color_elem is not None:
                    rgba = color_elem.get("rgba", "0.8 0.8 0.8 1")
                    geom.set("rgba", rgba)

    def _add_collision_geom(self, collision: ET.Element, body: ET.Element, link_name: str, base_path: Path):
        """Add collision geometry"""
        geometry = collision.find("geometry")
        if geometry is None:
            return

        geom = ET.SubElement(body, "geom")
        base_geom_name = collision.get("name", f"{link_name}_collision")
        geom_name = self._ensure_unique_geom_name(base_geom_name)
        geom.set("name", geom_name)
        geom.set("type", "mesh")
        geom.set("group", "3")  # Collision group
        geom.set("rgba", "0.5 0.5 0.5 0.3")  # Semi-transparent

        # Get origin
        origin = collision.find("origin")
        if origin is not None:
            xyz = origin.get("xyz", "0 0 0")
            geom.set("pos", xyz)

            rpy = origin.get("rpy", "0 0 0")
            if rpy != "0 0 0":
                quat = self._rpy_to_quat(rpy)
                geom.set("quat", quat)

        # Get geometry type
        self._add_geometry(geometry, geom)

    def _add_geometry(self, geometry: ET.Element, geom: ET.Element):
        """Add geometry parameters to geom element"""
        # Check for primitive types
        box = geometry.find("box")
        if box is not None:
            geom.set("type", "box")
            size = box.get("size", "0.1 0.1 0.1")
            # MuJoCo uses half-sizes
            half_sizes = " ".join(str(float(s)/2) for s in size.split())
            geom.set("size", half_sizes)
            return

        cylinder = geometry.find("cylinder")
        if cylinder is not None:
            geom.set("type", "cylinder")
            radius = cylinder.get("radius", "0.05")
            length = cylinder.get("length", "0.1")
            geom.set("size", f"{radius} {float(length)/2}")
            return

        sphere = geometry.find("sphere")
        if sphere is not None:
            geom.set("type", "sphere")
            radius = sphere.get("radius", "0.05")
            geom.set("size", radius)
            return

        mesh = geometry.find("mesh")
        if mesh is not None:
            geom.set("type", "mesh")
            filename = mesh.get("filename", "")
            if filename:
                if filename.startswith("package://"):
                    filename = filename.replace("package://", "")
                mesh_name = Path(filename).stem
                geom.set("mesh", mesh_name)

    def _convert_children(self, urdf_root: ET.Element, parent_link: str, parent_body: ET.Element, base_path: Path):
        """Recursively convert child links and joints"""
        for joint in urdf_root.findall("joint"):
            parent_elem = joint.find("parent")
            if parent_elem is not None and parent_elem.get("link") == parent_link:
                # This joint connects to our parent link
                child_elem = joint.find("child")
                if child_elem is None:
                    continue

                child_link_name = child_elem.get("link")
                child_link = urdf_root.find(f".//link[@name='{child_link_name}']")

                if child_link is not None:
                    # Create child body
                    child_body = ET.SubElement(parent_body, "body")
                    child_body.set("name", child_link_name)

                    # Get joint origin
                    origin = joint.find("origin")
                    if origin is not None:
                        xyz = origin.get("xyz", "0 0 0")
                        child_body.set("pos", xyz)

                        rpy = origin.get("rpy", "0 0 0")
                        if rpy != "0 0 0":
                            quat = self._rpy_to_quat(rpy)
                            child_body.set("quat", quat)

                    # Add joint
                    self._add_joint(joint, child_body)

                    # Convert child link
                    self._convert_link(child_link, child_body, base_path)

                    # Recursively convert grandchildren
                    self._convert_children(urdf_root, child_link_name, child_body, base_path)

    def _add_joint(self, joint: ET.Element, body: ET.Element):
        """Add joint to body"""
        joint_name = joint.get("name")
        joint_type = joint.get("type")

        # Create MuJoCo joint
        if joint_type == "fixed":
            # Fixed joints don't need explicit joint element in MuJoCo
            return

        mj_joint = ET.SubElement(body, "joint")
        mj_joint.set("name", joint_name)

        # Convert joint type
        if joint_type == "revolute" or joint_type == "continuous":
            mj_joint.set("type", "hinge")
        elif joint_type == "prismatic":
            mj_joint.set("type", "slide")
        elif joint_type == "floating":
            mj_joint.set("type", "free")
        elif joint_type == "planar":
            # Planar not directly supported, use slide + hinge
            self.logger.warning(f"Planar joint {joint_name} approximated with slide")
            mj_joint.set("type", "slide")

        # Get axis
        axis = joint.find("axis")
        if axis is not None:
            xyz = axis.get("xyz", "1 0 0")
            mj_joint.set("axis", xyz)

        # Get limits
        limit = joint.find("limit")
        if limit is not None and joint_type != "continuous":
            lower = limit.get("lower", "-3.14")
            upper = limit.get("upper", "3.14")
            mj_joint.set("range", f"{lower} {upper}")

            effort = limit.get("effort", "100")
            velocity = limit.get("velocity", "10")
            # These will be used in actuators

        # Get dynamics
        dynamics = joint.find("dynamics")
        if dynamics is not None:
            damping = dynamics.get("damping", "0.1")
            friction = dynamics.get("friction", "0")
            mj_joint.set("damping", damping)
            if float(friction) > 0:
                mj_joint.set("frictionloss", friction)

    def _add_actuators(self, urdf_root: ET.Element, actuator: ET.Element):
        """Add actuators for all non-fixed joints"""
        for joint in urdf_root.findall("joint"):
            joint_type = joint.get("type")
            joint_name = joint.get("name")

            if joint_type in ["revolute", "continuous", "prismatic"]:
                # Add position actuator
                motor = ET.SubElement(actuator, "position")
                motor.set("name", f"{joint_name}_motor")
                motor.set("joint", joint_name)
                motor.set("kp", "10")  # Position gain

                # Get effort limit if available
                limit = joint.find("limit")
                if limit is not None:
                    effort = limit.get("effort", "100")
                    motor.set("ctrlrange", f"-{effort} {effort}")

    def _add_sensors(self, urdf_root: ET.Element, sensor: ET.Element):
        """Add sensors for all joints"""
        for joint in urdf_root.findall("joint"):
            joint_type = joint.get("type")
            joint_name = joint.get("name")

            if joint_type in ["revolute", "continuous", "prismatic"]:
                # Add joint position sensor
                pos_sensor = ET.SubElement(sensor, "jointpos")
                pos_sensor.set("name", f"{joint_name}_pos")
                pos_sensor.set("joint", joint_name)

                # Add joint velocity sensor
                vel_sensor = ET.SubElement(sensor, "jointvel")
                vel_sensor.set("name", f"{joint_name}_vel")
                vel_sensor.set("joint", joint_name)

    def _rpy_to_quat(self, rpy_str: str) -> str:
        """Convert roll-pitch-yaw to quaternion string"""
        try:
            r, p, y = map(float, rpy_str.split())

            # Convert RPY to quaternion (w, x, y, z order for MuJoCo)
            cy = np.cos(y * 0.5)
            sy = np.sin(y * 0.5)
            cp = np.cos(p * 0.5)
            sp = np.sin(p * 0.5)
            cr = np.cos(r * 0.5)
            sr = np.sin(r * 0.5)

            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy

            return f"{w} {x} {y} {z}"
        except:
            self.logger.warning(f"Failed to convert RPY '{rpy_str}' to quaternion")
            return "1 0 0 0"


def convert_urdf_to_mjcf(urdf_path: str, output_path: Optional[str] = None) -> str:
    """
    Convenience function to convert URDF to MJCF

    Args:
        urdf_path: Path to input URDF file
        output_path: Path for output MJCF file (optional)

    Returns:
        Path to generated MJCF file
    """
    converter = URDFToMJCFConverter()
    return converter.convert(urdf_path, output_path)


if __name__ == "__main__":
    # Test conversion
    import sys

    if len(sys.argv) < 2:
        print("Usage: python urdf_to_mjcf.py <urdf_file> [output_file]")
        sys.exit(1)

    urdf_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    logging.basicConfig(level=logging.INFO)

    try:
        result = convert_urdf_to_mjcf(urdf_file, output_file)
        print(f"✓ Conversion successful: {result}")
    except Exception as e:
        print(f"✗ Conversion failed: {e}")
        sys.exit(1)
