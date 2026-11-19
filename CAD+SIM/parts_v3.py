import math
from dataclasses import dataclass, field
from pathlib import Path
from shutil import copyfile
from concurrent.futures import ProcessPoolExecutor

import cadquery as cq
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps


@dataclass
class PartSummary:
    name: str
    mass_g: float
    copies: int
    total_mass_g: float


@dataclass
class PendulumParameters:
    """Input dimensions, print settings, and simulation options for one kit.

    CAD dimensions use millimetres; density is in g/cm^3. Simulation
    settings such as the initial angle and duration use radians and seconds.
    """

    # Main design inputs. The arm hangs along -Z, with the axle on X.
    arm_length: float = 120.0
    axle_diameter: float = 5.0
    max_height: float = 250.0
    order_id: str = "12"

    # Print estimates are calculated separately for walls and infill.
    wall_loops: int = 2
    line_width: float = 0.2
    infill: dict = field(default_factory=lambda: {
        "base": 0.20, "support": 0.20, "arm": 0.15,
        "weight": 1.00, "axle": 1.00, "spacer": 1.00,
    })
    filament: dict = field(default_factory=lambda: {
        "base": "PLA+", "support": "PLA+", "arm": "PLA+",
        "weight": "PETG", "axle": "PLA+", "spacer": "PLA+",
    })
    density: dict = field(default_factory=lambda: {
        "PLA": 1.24, "PLA+": 1.24, "PLA+HS": 1.24, "PETG": 1.27, "ABS+": 1.04,
    })
    copies: dict = field(default_factory=lambda: {
        "base": 1, "support": 2, "arm": 1,
        "weight": 1, "axle": 1, "spacer": 2,
    })

    # Fits and clearances are total differences in the stated dimensions.
    fit: float = 0.3
    support_clearance: float = 0.3
    arm_clearance: float = 1.0
    press_fit: float = 0.15
    gap: float = 3.0
    base_clearance: float = 5.0
    axle_overhang: float = 3.0
    spacer_play: float = 0.5

    # Part dimensions, in millimetres.
    arm_thickness: float = 6.0
    arm_width: float = 12.0
    hub_wall: float = 4.5
    weight_thickness: float = 20.0
    weight_width: float = 30.0
    weight_height: float = 30.0
    slot_depth: float = 20.0
    beam_thickness: float = 12.0
    beam_width: float = 25.0
    top_margin: float = 10.0
    base_thickness: float = 20.0
    base_width: float = 150.0
    base_margin: float = 40.0
    socket_depth: float = 15.0

    output_dir: str = "sim_model"
    video_path: str = ""
    colors: dict = field(default_factory=lambda: {
        "base": "1 1 0 1",
        "support_R": "1 0 0 1",
        "support_L": "1 0 0 1",
        "axle": "1 0 0 1",
        "spacer_R": "1 0 0 1",
        "spacer_L": "1 0 0 1",
        "arm": "1 0 0 1",
        "weight": "0 0 1 1",
    })

    initial_angle_rad: float = math.pi / 0.9
    simulation_duration: float = 10.0
    timestep: float = 0.0002
    gravity: float = 9.81
    fps: int = 30
    video_width: int = 640
    video_height: int = 480
    friction_factor: float = 0.01
    damping_factor: float = 0.004
    show_plot: bool = False
    target_period: float | None = None


class PendulumKit:
    """Build and simulate one pendulum kit."""

    def __init__(self, parameters=None):
        self.parameters = parameters or PendulumParameters()
        self.built = False
        self.exported = False
        self.period_to_show = None
        self.delta = None
        self.filament_use = None
        self.summary: dict[str, PartSummary] = {}

    @staticmethod
    def _geometry_properties(workplane):
        """Return volume, centre, and inertia properties for all solids."""
        properties = GProp_GProps()
        solids = cq.Compound.makeCompound(workplane.solids().vals())
        BRepGProp.VolumeProperties_s(solids.wrapped, properties)
        return properties

    def build(self):
        p = self.parameters
        self.exported = False

        wall_thickness = p.wall_loops * p.line_width
        arm_hole = p.axle_diameter + p.arm_clearance
        hub_diameter = arm_hole + 2 * p.hub_wall
        support_hole = p.axle_diameter + p.support_clearance
        socket_x = p.beam_thickness + p.fit
        socket_y = p.beam_width + p.fit
        weight_top = -p.arm_length + p.slot_depth

        bar = cq.Workplane("XY").box(
            p.arm_thickness, p.arm_width, p.arm_length
        ).translate((0, 0, -p.arm_length / 2))
        hub = (
            cq.Workplane("YZ").circle(hub_diameter / 2).extrude(p.arm_thickness)
            .translate((-p.arm_thickness / 2, 0, 0))
        )
        arm = (
            bar.union(hub)
            .faces(">X").workplane(centerOption="ProjectedOrigin")
            .hole(arm_hole)
        )

        weight = (
            cq.Workplane("XY")
            .box(p.weight_thickness, p.weight_width, p.weight_height)
            .translate((0, 0, weight_top - p.weight_height / 2))
            .faces(">Z").workplane(centerOption="ProjectedOrigin")
            .rect(p.arm_thickness - p.press_fit, p.arm_width - p.press_fit)
            .cutBlind(-p.slot_depth)
        )

        bounds = cq.Compound.makeCompound([arm.val(), weight.val()]).BoundingBox()
        far_y = max(abs(bounds.ymin), abs(bounds.ymax))
        max_swing_radius = math.sqrt(bounds.zmin ** 2 + far_y ** 2)
        pivot_height = max_swing_radius + p.base_clearance
        half_x = max(abs(bounds.xmin), abs(bounds.xmax))
        beam_center_x = half_x + p.gap + p.beam_thickness / 2

        pivot_z = p.base_thickness / 2 + pivot_height
        beam_height = p.socket_depth + pivot_height + p.top_margin
        base_length = 2 * (
            beam_center_x + p.beam_thickness / 2 + p.base_margin
        )

        plate = (
            cq.Workplane("XY").box(base_length, p.base_width, p.base_thickness)
            .faces(">Z").workplane(centerOption="ProjectedOrigin")
            .pushPoints([(beam_center_x, 0), (-beam_center_x, 0)])
            .rect(socket_x, socket_y)
            .cutBlind(-p.socket_depth)
        )

        support = (
            cq.Workplane("XY").box(p.beam_thickness, p.beam_width, beam_height)
            .translate((0, 0, p.top_margin - beam_height / 2))
            .faces(">X").workplane(centerOption="ProjectedOrigin")
            .hole(support_hole)
        )

        axle_length = 2 * (
            beam_center_x + p.beam_thickness / 2 + p.axle_overhang
        )
        axle = (
            cq.Workplane("YZ").circle(p.axle_diameter / 2).extrude(axle_length)
            .translate((-axle_length / 2, 0, 0))
        )

        opening = 2 * (beam_center_x - p.beam_thickness / 2)
        spacer_length = (opening - p.arm_thickness - p.spacer_play) / 2
        spacer = (
            cq.Workplane("YZ")
            .circle(hub_diameter / 2).circle(arm_hole / 2)
            .extrude(spacer_length)
        )

        self.parts = {
            "base": plate, "support": support, "arm": arm,
            "weight": weight, "axle": axle, "spacer": spacer,
        }
        self.walls = {
            name: part.shell(-wall_thickness)
            for name, part in self.parts.items()
        }
        self.core = {
            name: part.cut(self.walls[name])
            for name, part in self.parts.items()
        }

        self.properties = {}
        self.summary = {}
        for name in self.parts:
            rho = p.density[p.filament[name]] * 1e-3
            properties = GProp_GProps()
            properties.Add(self._geometry_properties(self.walls[name]), rho)
            properties.Add(
                self._geometry_properties(self.core[name]),
                rho * p.infill[name],
            )
            self.properties[name] = properties

            mass = float(properties.Mass())
            num_copies = int(p.copies[name])
            self.summary[name] = PartSummary(
                name=name,
                mass_g=mass,
                copies=num_copies,
                total_mass_g=mass * num_copies,
            )
            print(f"{name:8s} {mass:7.2f} g  x{num_copies}")

        filament_use = {}
        for name, item in self.summary.items():
            filament_name = p.filament[name]
            filament_use[filament_name] = (
                filament_use.get(filament_name, 0.0) + item.total_mass_g
            )
        print(
            "filament use:",
            {name: round(mass, 1) for name, mass in filament_use.items()},
            "g",
        )
        self.filament_use = filament_use

        spacer_x = p.arm_thickness / 2 + p.spacer_play / 2
        self.assembly = {
            "plate": plate,
            "support_L": support.translate((-beam_center_x, 0, pivot_z)),
            "support_R": support.translate((beam_center_x, 0, pivot_z)),
            "arm": arm.translate((0, 0, pivot_z)),
            "weight": weight.translate((0, 0, pivot_z)),
            "axle": axle.translate((0, 0, pivot_z)),
            "spacer_L": spacer.translate(
                (-spacer_x - spacer_length, 0, pivot_z)
            ),
            "spacer_R": spacer.translate((spacer_x, 0, pivot_z)),
        }

        self.dimensions = {
            "pivot_height": pivot_height,
            "pivot_z": pivot_z,
            "beam_center_x": beam_center_x,
            "beam_height": beam_height,
            "base_length": base_length,
            "spacer_length": spacer_length,
            "spacer_x": spacer_x,
            "total_height": p.base_thickness + pivot_height + p.top_margin,
        }
        self.meshes = {
            "base": plate.translate((0, 0, -pivot_z)),
            "support_R": support.translate((beam_center_x, 0, 0)),
            "support_L": support.translate((-beam_center_x, 0, 0)),
            "axle": axle,
            "spacer_R": spacer.translate((spacer_x, 0, 0)),
            "spacer_L": spacer.translate(
                (-spacer_x - spacer_length, 0, 0)
            ),
            "arm": arm,
            "weight": weight,
        }

        total_height = self.dimensions["total_height"]
        print(
            f"h_pivot = {pivot_height:.2f} mm   "
            f"beam_cx = {beam_center_x:.2f} mm   "
            f"beam_h = {beam_height:.2f} mm   "
            f"base_l = {base_length:.1f} mm"
        )
        print(
            f"total height = {total_height:.1f} mm "
            f"(H_max = {p.max_height:.0f}): "
            f"{'OK' if total_height <= p.max_height else 'TOO TALL'}"
        )

        self.built = True
        return self

    def export(self):
        if not self.built:
            self.build()

        p = self.parameters
        output_dir = Path(p.output_dir)
        meshes_dir = output_dir / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)

        for name, workplane in self.meshes.items():
            cq.exporters.export(workplane, str(meshes_dir / f"{name}.stl"))
        for name, workplane in self.meshes.items():
            cq.exporters.export(workplane, str(meshes_dir / f"{name}.3mf"))

        swing = GProp_GProps()
        swing.Add(self.properties["arm"])
        swing.Add(self.properties["weight"])
        mass_kg = swing.Mass() * 1e-3
        center = swing.CentreOfMass()
        inertia = swing.MatrixOfInertia()
        center_of_mass = (
            f"{center.X() * 1e-3:.6g} "
            f"{center.Y() * 1e-3:.6g} "
            f"{center.Z() * 1e-3:.6g}"
        )
        diagonal_inertia = " ".join(
            f"{inertia.Value(i, i) * 1e-9:.6g}" for i in (1, 2, 3)
        )

        colors = p.colors
        assets = "\n".join(
            f'    <mesh name="{name}" file="{name}.stl" '
            'scale="0.001 0.001 0.001"/>'
            for name in self.meshes
        )
        static_geometries = "\n".join(
            f'    <geom mesh="{name}" rgba="{colors[name]}"/>'
            for name in self.meshes
            if name not in ("arm", "weight")
        )

        length = p.arm_length
        distance = 1 / 320 * (length - 220) + 1
        camera_coordinates = (
            f"{0.5 * distance} {-0.5 * distance} {-0.05 * distance}"
        )
        camera = (
            f'  <camera name="cam" pos="{camera_coordinates}" '
            'xyaxes="1 1 0  0 0 1"/>'
        )
        xml = f"""<mujoco model="pendulum_kit">
                <compiler meshdir="meshes"/>
                <option timestep="{p.timestep}" gravity="0 0 {-p.gravity}" integrator="implicitfast"/>
                <asset>
                {assets}
                </asset>
                <default>
                <geom type="mesh" contype="0" conaffinity="0"/>
                </default>
                <worldbody>
                <light pos="0 -1 2" dir="0 0.5 -1"/>
                {camera}

                {static_geometries}
                <body name="pendulum">
                <joint name="axle" type="hinge" axis="1 0 0"/>
                <inertial pos="{center_of_mass}" mass="{mass_kg:.6g}" diaginertia="{diagonal_inertia}"/>
                <geom mesh="arm" rgba="{colors['arm']}"/>
                <geom mesh="weight" rgba="{colors['weight']}"/>
                </body>
                </worldbody>
                </mujoco>"""
        self.xml_path = output_dir / "pendulum.xml"
        self.xml_path.write_text(xml, encoding="utf-8")
        self.exported = True
        return self.xml_path

    def simulate(self):
        if not self.exported:
            self.export()

        import cv2
        import mujoco
        import matplotlib.pyplot as plt

        p = self.parameters
        model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        data = mujoco.MjData(model)
        data.qpos[0] = p.initial_angle_rad
        data.qvel[0] = 0

        renderer = mujoco.Renderer(
            model, height=p.video_height, width=p.video_width
        )
        times = []
        angles = []
        angular_velocities = []
        frames = []
        mass_kg = model.body_mass[model.body("pendulum").id]
        friction = p.friction_factor * mass_kg
        damping = p.damping_factor * mass_kg
        try:
            while data.time < p.simulation_duration:
                mujoco.mj_step(model, data)
                times.append(data.time)
                angles.append(data.qpos[0])
                angular_velocities.append(data.qvel[0])
                data.qfrc_applied[0] = (
                    -friction * np.sign(data.qvel[0])
                    - damping * data.qvel[0]
                )
                if len(frames) < data.time * p.fps:
                    renderer.update_scene(data, camera="cam")
                    frames.append(renderer.render().copy())
        finally:
            renderer.close()

        video_path = Path(
            p.video_path or f"Pendulum_{p.order_id}.mp4"
        )
        video_path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"avc1")
        video = cv2.VideoWriter(
            str(video_path), fourcc, p.fps, (p.video_width, p.video_height)
        )
        if not video.isOpened():
            raise RuntimeError(f"Could not open video writer for {video_path}")
        try:
            for frame in frames:
                video.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        finally:
            video.release()

        peak_times = [
            times[i]
            for i in range(1, len(angles) - 1)
            if angles[i - 1] < angles[i] > angles[i + 1]
        ]
        periods = [
            peak_times[i + 1] - peak_times[i]
            for i in range(len(peak_times) - 1)
        ]
        periods = [period for period in periods if period > 0.005]
        filtered_peak_times = [
            peak_times[i]
            for i in range(len(peak_times) - 1)
            if peak_times[i + 1] - peak_times[i] > 0.005
        ]
        print("Periods:", periods)
        self.period_to_show = (
            periods[3] if len(periods) > 3
            else periods[-1] if periods
            else None
        )
        print("Period to show:", self.period_to_show)
        if self.period_to_show is not None and p.target_period is not None:
            self.delta = abs(self.period_to_show - p.target_period)
            print(f"Delta (target={p.target_period:.4f} s): {self.delta:.4f} s")
        else:
            self.delta = None

        if p.show_plot:
            peak_angles = [
                angles[times.index(time)] for time in filtered_peak_times
            ]
            plt.plot(
                times, angles,
                times, angular_velocities,
                filtered_peak_times, peak_angles, "ro",
            )
            plt.xlabel("Time (s)")
            plt.ylabel("Pendulum Angle (rad) / Angular Velocity (rad/s)")
            plt.title("Pendulum Simulation")
            plt.legend(["Angle", "Angular Velocity", "Peak Angles"])
            plt.show()

        return periods

    def run(self):
        self.build()
        self.export()
        return self.simulate()


def _resolve_filament_config(params: dict) -> tuple[dict[str, str], dict[str, float]]:
    spool_to_material = {
        1: "PLA+",
        2: "PLA+",
        3: "PLA+",
        4: "PLA+",
        5: "PLA+",
        6: "PLA+",
        7: "PLA+HS",
        8: "ABS+",
    }
    density = {
        "PLA": 1.24,
        "PLA+": 1.24,
        "PLA+HS": 1.24,
        "PETG": 1.27,
        "ABS+": 1.04,
    }

    def resolve_one(key: str, default_material: str) -> str:
        raw = params.get(key)
        if raw is None or raw == "":
            return default_material
        if isinstance(raw, int) or (isinstance(raw, str) and raw.isdigit()):
            return spool_to_material.get(int(raw), default_material)
        return str(raw)

    filament = {
        "base": resolve_one("filament_base", "PLA+"),
        "support": resolve_one("filament_support", resolve_one("filament_stands", "PLA+")),
        "arm": resolve_one("filament_arm", "PLA+"),
        "weight": resolve_one("filament_weight", "PETG"),
        "axle": resolve_one("filament_axle", "PLA+"),
        "spacer": resolve_one("filament_spacer", "PLA+"),
    }

    for mat in filament.values():
        density.setdefault(mat, 1.24)

    return filament, density

def _resolve_colors(params: dict) -> dict[str, str]:
    filamentC = {
            1 : "1 0 0 1", #red
            2 : "0.5 0.5 0.5 1", #gray
            3 : "0.2 0.6 0.9 1", #blue
            4 : "1 1 1 1", #white
            5 : "0.2 0.2 0.2 1", #black
            6 : "0.95 0.9 0.4 1", #yellow
            7 : "0.2 0.6 0.9 1", #blue
            8 : "0.2 0.2 0.2 1", #black
        }

    base_c = filamentC.get(params.get("filament_base", "0 0 0.8 1"), params.get("filament_base", "0 0 0.8 1"))
    support_c = filamentC.get(params.get("filament_stands", "0.8 0.8 0.8 1"), params.get("filament_stands", "0.8 0.8 0.8 1"))
    axle_c = filamentC.get(params.get("filament_axle", "0.8 0.8 0.8 1"), params.get("filament_axle", "0.8 0.8 0.8 1"))
    spacer_c = filamentC.get(params.get("filament_stands", "0.8 0.8 0.8 1"), params.get("filament_stands", "0.8 0.8 0.8 1"))
    arm_c = filamentC.get(params.get("filament_arm", "0.2 0.6 0.9 1"), params.get("filament_arm", "0.2 0.6 0.9 1"))
    weight_c = filamentC.get(params.get("filament_weight", "0.9 0.6 0.2 1"), params.get("filament_weight", "0.9 0.6 0.2 1"))

    return {
            "base": base_c,
            "support_R": support_c,
            "support_L": support_c,
            "axle": axle_c,
            "spacer_R": spacer_c,
            "spacer_L": spacer_c,
            "arm": arm_c,
            "weight": weight_c,
        }


def simulate_pendulum(params: dict | None = None):
    params = params or {}
    order_id = str(params.get("order_id", "123456"))

    infill_raw = float(params.get("infill", 15.0))
    infill_pct = infill_raw / 100.0 if infill_raw > 1.0 else infill_raw

    filament, density = _resolve_filament_config(params)
    colors = _resolve_colors(params)

    raw_target = (
        params.get("swing_period")
        if params.get("swing_period") is not None
        else params.get("T_target", params.get("target_period"))
    )
    target_period = float(raw_target) if raw_target not in (None, "") else None

    pend_params = PendulumParameters(
        arm_length=float(params.get("arm_length", 120.0)),
        axle_diameter=float(params.get("axle_diameter", 5.0)),
        max_height=float(params.get("max_size", params.get("max_height", 250.0))),
        order_id=order_id,
        wall_loops=int(params.get("walls", params.get("wall_loops", 2))),
        infill={
            "base": infill_pct,
            "support": infill_pct,
            "arm": infill_pct,
            "weight": 1.00,
            "axle": 1.00,
            "spacer": 1.00,
        },
        filament=filament,
        density=density,
        colors=colors,
        output_dir=f"SIM_models/sim_model_{order_id}",
        video_path=f"static/Pendulum_{order_id}.mp4",
        target_period=target_period,
    )

    kit = PendulumKit(pend_params)
    periods = kit.run()
    return periods, kit.summary, kit.delta