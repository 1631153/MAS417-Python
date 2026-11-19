import math
from dataclasses import dataclass, field
from os import mkdir
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from concurrent.futures import ProcessPoolExecutor
from shutil import copyfile


@dataclass
class PendulumParameters:
    """Input dimensions, print settings, and simulation options for one kit.

    CAD dimensions use millimetres; density is in g/cm^3. Simulation
    settings such as the initial angle and duration use radians and seconds.
    """

    # Main design inputs. The arm hangs along -Z, with the axle on X.
    arm_length: float = 60.0
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
        "base": "PLA", "support": "PLA", "arm": "PLA",
        "weight": "PETG", "axle": "PLA", "spacer": "PLA",
    })
    density: dict = field(default_factory=lambda: {"PLA": 1.24, "PETG": 1.27})
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
    # Simulation and render controls. Use a distinct output directory for
    # every kit so their mesh and XML filenames do not overwrite each other.
    colors: dict = field(default_factory=lambda: {
            "base": "1 0 0 1",
            "support_R": "1 0 0 1",
            "support_L": "1 0 0 1",
            "axle": "1 0 0 1",
            "spacer_R": "1 0 0 1",
            "spacer_L": "1 0 0 1",
            "arm": "1 0 0 1",
            "weight": "0 0 1 1",
           })  # dict of RGBA strings for each part; default is all red except weight blue

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


class PendulumKit:
    """Build and simulate one pendulum kit.

    Create a separate instance and output directory for each parameter set.
    After running, the selected period is available as ``kit.period_to_show``.
    """

    def __init__(self, parameters=None):
        self.parameters = parameters or PendulumParameters()
        self.built = False
        self.exported = False
        self.period_to_show = None
        self.filament_use = None

    @staticmethod
    def _geometry_properties(workplane):
        """Return volume, centre, and inertia properties for all solids."""
        properties = GProp_GProps()
        solids = cq.Compound.makeCompound(workplane.solids().vals())
        BRepGProp.VolumeProperties_s(solids.wrapped, properties)
        return properties

    def build(self):
        # Build the pendulum kit parts and calculate their mass properties.
        p = self.parameters
        # A rebuild changes the geometry, so any previous XML export is stale.
        self.exported = False

        # Derive hole sizes and related dimensions from the user inputs.
        wall_thickness = p.wall_loops * p.line_width
        arm_hole = p.axle_diameter + p.arm_clearance
        hub_diameter = arm_hole + 2 * p.hub_wall
        support_hole = p.axle_diameter + p.support_clearance
        socket_x = p.beam_thickness + p.fit
        socket_y = p.beam_width + p.fit
        weight_top = -p.arm_length + p.slot_depth

        # Build the pendulum in its own frame: pivot at the origin, axle on X,
        # and the arm extending down the negative Z axis.
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

        # The weight pocket opens from the top and stops at the arm's lower end.
        weight = (
            cq.Workplane("XY")
            .box(p.weight_thickness, p.weight_width, p.weight_height)
            .translate((0, 0, weight_top - p.weight_height / 2))
            .faces(">Z").workplane(centerOption="ProjectedOrigin")
            .rect(p.arm_thickness - p.press_fit, p.arm_width - p.press_fit)
            .cutBlind(-p.slot_depth)
        )

        # Size the supports around the actual swinging assembly. The farthest
        # Y/Z corner must clear the base through a full swing.
        bounds = cq.Compound.makeCompound([arm.val(), weight.val()]).BoundingBox()
        far_y = max(abs(bounds.ymin), abs(bounds.ymax))
        max_swing_radius = math.sqrt(bounds.zmin ** 2 + far_y ** 2)
        pivot_height = max_swing_radius + p.base_clearance
        half_x = max(abs(bounds.xmin), abs(bounds.xmax))
        beam_center_x = half_x + p.gap + p.beam_thickness / 2

        # Convert the pivot height above the plate into the plate's Z frame.
        pivot_z = p.base_thickness / 2 + pivot_height
        beam_height = p.socket_depth + pivot_height + p.top_margin
        base_length = 2 * (
            beam_center_x + p.beam_thickness / 2 + p.base_margin
        )

        # Cut two beam sockets into the base, symmetric about the axle centre.
        plate = (
            cq.Workplane("XY").box(base_length, p.base_width, p.base_thickness)
            .faces(">Z").workplane(centerOption="ProjectedOrigin")
            .pushPoints([(beam_center_x, 0), (-beam_center_x, 0)])
            .rect(socket_x, socket_y)
            .cutBlind(-p.socket_depth)
        )

        # Supports are modelled around the pivot origin; their lower ends
        # extend into the base sockets when assembled.
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

        # Split the available gap evenly between the two spacers, allowing
        # for the arm thickness and the requested axial play.
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
        # Shelling inward gives the solid outer walls; the remaining volume is
        # the region whose effective density is reduced by the infill setting.
        self.walls = {
            name: part.shell(-wall_thickness)
            for name, part in self.parts.items()
        }
        self.core = {
            name: part.cut(self.walls[name])
            for name, part in self.parts.items()
        }

        # Combine wall and core mass properties using density in g/mm^3.
        self.properties = {}
        for name in self.parts:
            density = p.density[p.filament[name]] * 1e-3
            properties = GProp_GProps()
            properties.Add(self._geometry_properties(self.walls[name]), density)
            properties.Add(
                self._geometry_properties(self.core[name]),
                density * p.infill[name],
            )
            self.properties[name] = properties
            print(
                f"{name:8s} {properties.Mass():7.2f} g  "
                f"x{p.copies[name]}"
            )

        filament_use = {}
        for name, properties in self.properties.items():
            filament_name = p.filament[name]
            filament_use[filament_name] = (
                filament_use.get(filament_name, 0.0)
                + p.copies[name] * properties.Mass()
            )
        print(
            "filament use:",
            {name: round(mass, 1) for name, mass in filament_use.items()},
            "g",
        )
        self.filament_use = filament_use

        # Keep assembly placements (in the plate frame) separate from the
        # pivot-frame meshes written for MuJoCo.
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

        # Report the overall height against the configured print/kit limit.
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
        # Write the pendulum meshes to STL files and create a MuJoCo XML file for simulation.
        if not self.built:
            self.build()

        p = self.parameters
        # MuJoCo resolves the STL filenames relative to the XML's meshdir.
        output_dir = Path(p.output_dir)
        meshes_dir = output_dir / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)

        for name, workplane in self.meshes.items():
            cq.exporters.export(workplane, str(meshes_dir / f"{name}.stl"))
        #Export 3mf-files for 3D printing. 
        for name, workplane in self.meshes.items():
            cq.exporters.export(workplane, str(meshes_dir / f"{name}.3mf"))
        # The arm and weight form one moving body. Convert CAD units (g, mm)
        # to MuJoCo units (kg, m) for mass, centre of mass, and inertia.
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
        print(
            "off-diagonal (should be ~0):",
            inertia.Value(1, 2), inertia.Value(1, 3), inertia.Value(2, 3),
        )

        colors = p.colors
        # Create XML asset and geometry entries from the same mesh-name map
        # used above when writing the STL files.
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

        # Approximate camera distance from arm length to keep different-sized
        # designs framed similarly in the rendered video.
        length = p.arm_length
        distance = 1 / 320 * (length - 220) + 1
        camera_coordinates = (
            f"{0.5 * distance} {-0.5 * distance} {-0.05 * distance}"
        )
        # Aim the camera towards the origin, with X/Y axes in the image plane and Z pointing up. Angled at 45 degrees.
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
        # Run the MuJoCo simulation and render a video of the pendulum's motion.
        if not self.exported:
            self.export()

        import cv2
        import mujoco
        import matplotlib.pyplot as plt

        p = self.parameters
        # Initialize the hinge angle and collect values/frames as the model
        # advances through the configured simulation duration.
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
        # Apply Coulomb friction and velocity-proportional damping as a hinge
        # torque. Render only when enough simulation time has elapsed for the
        # next output frame at the requested FPS.
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

        # MuJoCo renders RGB images; OpenCV's VideoWriter expects BGR images.
        video_path = Path(
            p.video_path or f"Pendulum_{p.order_id}.mp4"
        )
        video_path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
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

        # A local maximum marks one positive-angle peak. Consecutive peaks
        # estimate the period; discard very short intervals as numerical noise.
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


def _run_kit(parameters):
   """Build and simulate one kit, returning its period and filament use."""
   kit = PendulumKit(parameters)
   kit.run()
   return kit.period_to_show, kit.filament_use
   

# for the final video render, run all three kits in parallel and select the one with the period closest to the reference value.
def run_simulation():
   model_id = "15" 
   max_L = 200
   L = np.linspace(100, max_L, 3)
   
   period_ref = 1.0
   colors = {
            "base": "1 0 1 1",
            "support_R": "1 0 0 1",
            "support_L": "1 0 0 1",
            "axle": "1 0 0 1",
            "spacer_R": "1 0 0 1",
            "spacer_L": "1 0 0 1",
            "arm": "1 0 0 1",
            "weight": "0 0 1 1",
           }
   parameters = [
       PendulumParameters(
           order_id=model_id, arm_length=L[0],
           output_dir=f"sim_model_{model_id}A", video_path=f"Pendulum_{model_id}A.mp4",
           colors=colors,
       ),
       PendulumParameters(
           order_id=model_id, arm_length=L[1],
           output_dir=f"sim_model_{model_id}B", video_path=f"Pendulum_{model_id}B.mp4",
           colors=colors,
       ),
       PendulumParameters(
           order_id=model_id, arm_length=L[2],
           output_dir=f"sim_model_{model_id}C", video_path=f"Pendulum_{model_id}C.mp4",
           colors=colors,
       ),
   ]
   # Separate processes avoid shared CadQuery parser and MuJoCo GL contexts.
   with ProcessPoolExecutor(max_workers=len(parameters)) as executor:
       results = list(executor.map(_run_kit, parameters))

   (period_A, filament_use_A) = results[0]
   (period_B, filament_use_B) = results[1]
   (period_C, filament_use_C) = results[2]

   delta_A = abs(period_A - period_ref)
   delta_B = abs(period_B - period_ref)
   delta_C = abs(period_C - period_ref)
   #Choose model with lowest delta from reference period for final video render.
   if delta_A <= delta_B and delta_A <= delta_C:
        print("Model A has the closest period to reference.")
        copyfile(f"Pendulum_{model_id}A.mp4", f"Pendulum_{model_id}.mp4")
        print(f"Pendulum_{model_id}.mp4 created from Model A.")
        print(f"Period A: {period_A:.4f} s, Delta: {delta_A:.4f} s")
        #Copying all 3mf files to Final_mesh folder for 3D printing.
        copyfile(f"sim_model_{model_id}A/meshes/base.3mf", f"Final_mesh/base.3mf")
        copyfile(f"sim_model_{model_id}A/meshes/support_L.3mf", f"Final_mesh/support_L.3mf")
        copyfile(f"sim_model_{model_id}A/meshes/support_R.3mf", f"Final_mesh/support_R.3mf")
        copyfile(f"sim_model_{model_id}A/meshes/axle.3mf", f"Final_mesh/axle.3mf")
        copyfile(f"sim_model_{model_id}A/meshes/arm.3mf", f"Final_mesh/arm.3mf")
        copyfile(f"sim_model_{model_id}A/meshes/weight.3mf", f"Final_mesh/weight.3mf")
        copyfile(f"sim_model_{model_id}A/meshes/spacer_L.3mf", f"Final_mesh/spacer_L.3mf")
        copyfile(f"sim_model_{model_id}A/meshes/spacer_R.3mf", f"Final_mesh/spacer_R.3mf")
        best_filament_use = filament_use_A
        delta = delta_A
   elif delta_B <= delta_A and delta_B <= delta_C:
        print("Model B has the closest period to reference.")
        copyfile(f"Pendulum_{model_id}B.mp4", f"Pendulum_{model_id}.mp4")
        print(f"Pendulum_{model_id}.mp4 created from Model B.")
        print(f"Period B: {period_B:.4f} s, Delta: {delta_B:.4f} s")
        #Copying all 3mf files to Final_mesh folder for 3D printing.
        copyfile(f"sim_model_{model_id}B/meshes/base.3mf", f"Final_mesh/base.3mf")
        copyfile(f"sim_model_{model_id}B/meshes/support_L.3mf", f"Final_mesh/support_L.3mf")
        copyfile(f"sim_model_{model_id}B/meshes/support_R.3mf", f"Final_mesh/support_R.3mf")
        copyfile(f"sim_model_{model_id}B/meshes/axle.3mf", f"Final_mesh/axle.3mf")
        copyfile(f"sim_model_{model_id}B/meshes/arm.3mf", f"Final_mesh/arm.3mf")
        copyfile(f"sim_model_{model_id}B/meshes/weight.3mf", f"Final_mesh/weight.3mf")
        copyfile(f"sim_model_{model_id}B/meshes/spacer_L.3mf", f"Final_mesh/spacer_L.3mf")
        copyfile(f"sim_model_{model_id}B/meshes/spacer_R.3mf", f"Final_mesh/spacer_R.3mf")
        best_filament_use = filament_use_B
        delta = delta_B
   else:
    
        print("Model C has the closest period to reference.")
        copyfile(f"Pendulum_{model_id}C.mp4", f"Pendulum_{model_id}.mp4")
        print(f"Pendulum_{model_id}.mp4 created from Model C.")
        print(f"Period C: {period_C:.4f} s, Delta: {delta_C:.4f} s")
        #Copying all 3mf files to Final_mesh folder for 3D printing.
        copyfile(f"sim_model_{model_id}C/meshes/base.3mf", f"Final_mesh/base.3mf")
        copyfile(f"sim_model_{model_id}C/meshes/support_L.3mf", f"Final_mesh/support_L.3mf")
        copyfile(f"sim_model_{model_id}C/meshes/support_R.3mf", f"Final_mesh/support_R.3mf")
        copyfile(f"sim_model_{model_id}C/meshes/axle.3mf", f"Final_mesh/axle.3mf")
        copyfile(f"sim_model_{model_id}C/meshes/arm.3mf", f"Final_mesh/arm.3mf")
        copyfile(f"sim_model_{model_id}C/meshes/weight.3mf", f"Final_mesh/weight.3mf")
        copyfile(f"sim_model_{model_id}C/meshes/spacer_L.3mf", f"Final_mesh/spacer_L.3mf")
        copyfile(f"sim_model_{model_id}C/meshes/spacer_R.3mf", f"Final_mesh/spacer_R.3mf")
        best_filament_use = filament_use_C
        delta = delta_C

   total_filament_use = sum(best_filament_use.values())
   material_totals = ", ".join(
       f"{material}: {mass:.1f} g"
       for material, mass in best_filament_use.items()
   )
   print(
       f"Best model filament use: {total_filament_use:.1f} g total "
       f"({material_totals})"
   )
   return "Pendulum_{model_id}.mp4".format(model_id=model_id), delta, total_filament_use, best_filament_use
   

   

    


def main():
    # Create the Final_mesh directory if it doesn't exist
    final_mesh_dir = Path("Final_mesh")
    final_mesh_dir.mkdir(parents=True, exist_ok=True)

    result_message = run_simulation()
    print(result_message)


if __name__ == "__main__":
    main()
