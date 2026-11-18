import math

import cadquery as cq

import numpy as np
from cadquery.vis import show

from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Ax1, gp_Pnt, gp_Dir

import cv2
import matplotlib.pyplot as plt

def simulate_pendulum(params):
    # =====================================================================
    # 1. PARAMETERS  (later read from params.json)
    # =====================================================================
    # Customer order
    L        = 120        # arm length, pivot to arm end [mm] (later solved from the swing period)
    axle_d   = float(params.get("axle_diameter", 5.0))          # axle diameter [mm]
    H_max    = float(params.get("max_size", 250.0))        # max total height of the kit [mm]
    order_id = str(params.get("order_id", "123456"))      # customer order number

    # Print settings
    n_walls  = int(params.get("walls", 2))           # wall loops
    infill_raw = float(params.get("infill", 15))
    infill_pct = infill_raw / 100
    line_w   = 0.2          # extrusion line width [mm]
    infill = {"base": infill_pct, "support": infill_pct, "arm": infill_pct, "weight": 1.00, "axle": 1.00, "spacer": 1.00}

    filamentN = {
                1 : "PLA+",
                2 : "PLA+",
                3 : "PLA+",
                4 : "PLA+",
                5 : "PLA+",
                6 : "PLA+",
                7 : "PLA+HS",
                8 : "ABS+",
            }
    
    # Material: filament per part, density per filament (from Spoolman, g/cm^3)
    filament = {"base": filamentN.get(params.get("filament_base", "PLA+"), params.get("filament_base", "PLA+")),
                "support": filamentN.get(params.get("filament_support", "PLA+"), params.get("filament_support", "PLA+")),
                "arm": filamentN.get(params.get("filament_arm", "PLA+"), params.get("filament_arm", "PLA+")),
                "weight": filamentN.get(params.get("filament_weight", "PETG"), params.get("filament_weight", "PETG")),
                "axle": filamentN.get(params.get("filament_axle", "PLA+"), params.get("filament_axle", "PLA+")),
                "spacer": "PLA+"}
    density  = {"PLA+": 1.24, "PLA+HS": 1.24, "PETG": 1.27, "ABS+": 1.04}
    copies   = {"base": 1, "support": 2, "arm": 1, "weight": 1, "axle": 1, "spacer": 2}

    # Fits and clearances [mm]
    fit         = 0.3       # beam in socket, total per dimension
    support_clr = 0.3       # axle in the support holes
    arm_clr     = 1.0       # axle in the arm hole (free swing)
    press       = 0.15      # arm in the weight pocket (interference)
    gap         = 3.0       # free space between the weight and each beam
    c_base      = 5.0       # pendulum to base top, at full swing
    axle_overhang = 3.0     # axle sticks out past each support [mm]
    spacer_play = 0.5       # total axial play for the arm between the spacers [mm]


    # Arm
    arm_t    = 6.0          # thickness along the axle (X)
    arm_w    = 12.0         # width in the swing plane (Y)
    hub_wall = 4.5          # material around the arm hole

    # Weight
    weight_t = 20.0         # thickness along the axle (X)
    weight_w = 30.0         # width in the swing plane (Y)
    weight_h = 30.0         # height along the arm (Z)
    slot_d   = 20.0         # how far the arm goes into the weight

    # Supports (beams)
    beam_x   = 12.0         # thickness along the axle (X)
    beam_y   = 25.0         # width across (Y)
    top_mrg  = 10.0         # material above the axle centre

    # Base plate
    base_t      = 20.0      # thickness (Z)
    base_w      = 150.0     # size in the swing direction (Y), for stability
    base_margin = 40.0      # plate edge beyond each beam (X)
    socket_d    = 15.0      # socket depth

    # =====================================================================
    # 2. DERIVED DIMENSIONS (from the parameters only)
    # =====================================================================
    t_wall       = n_walls * line_w             # 2 * 0.45 = 0.9 mm
    arm_hole     = axle_d + arm_clr             # 5 + 1 = 6.0 mm
    hub_d        = arm_hole + 2 * hub_wall      # 6 + 9 = 15.0 mm
    support_hole = axle_d + support_clr         # 5 + 0.3 = 5.3 mm
    sock_dx      = beam_x + fit                 # 12.3 mm (socket larger than the beam)
    sock_dy      = beam_y + fit                 # 25.3 mm
    z_top        = -L + slot_d                  # weight top face in the pivot frame: -140 mm

    # =====================================================================
    # 3. PENDULUM  (frame: origin = pivot, axle along X, hangs along -Z)
    # =====================================================================
    # Arm: bar from the pivot down to z = -L, hub disc around the pivot
    bar = cq.Workplane("XY").box(arm_t, arm_w, L).translate((0, 0, -L / 2))
    hub = (cq.Workplane("YZ").circle(hub_d / 2).extrude(arm_t)
            .translate((-arm_t / 2, 0, 0)))
    arm = (bar.union(hub)
            .faces(">X").workplane(centerOption="ProjectedOrigin")
            .hole(arm_hole))                                  # through, at the pivot

    # Weight: block with a press-fit pocket; the arm end rests on the pocket floor (z = -L)
    weight = (cq.Workplane("XY").box(weight_t, weight_w, weight_h)
                .translate((0, 0, z_top - weight_h / 2))        # top face at z_top
                .faces(">Z").workplane(centerOption="ProjectedOrigin")
                .rect(arm_t - press, arm_w - press)             # pocket: X first, then Y
                .cutBlind(-slot_d))                             # down to z = -L

    # =====================================================================
    # 4. MEASURE THE PENDULUM  -> support height and spacing, plate size
    # =====================================================================
    bb      = cq.Compound.makeCompound([arm.val(), weight.val()]).BoundingBox()
    y_far   = max(abs(bb.ymin), abs(bb.ymax))
    r_max   = math.sqrt(bb.zmin**2 + y_far**2)          # farthest point from the pivot (swing)
    h_pivot = r_max + c_base                            # axle height above the base top
    x_half  = max(abs(bb.xmin), abs(bb.xmax))           # widest part along the axle
    beam_cx = x_half + gap + beam_x / 2                 # beam centre, from the middle

    pivot_z = base_t / 2 + h_pivot                      # axle height in the plate frame
    beam_h  = socket_d + h_pivot + top_mrg              # full beam length
    base_l  = 2 * (beam_cx + beam_x / 2 + base_margin)  # plate size along the axle (X)

    # =====================================================================
    # 5. BASE PLATE  (frame: centred, top face at z = +base_t/2)
    # =====================================================================
    plate = (cq.Workplane("XY").box(base_l, base_w, base_t)
            .faces(">Z").workplane(centerOption="ProjectedOrigin")
            .pushPoints([(beam_cx, 0), (-beam_cx, 0)])
            .rect(sock_dx, sock_dy)
            .cutBlind(-socket_d))

    # =====================================================================
    # 6. SUPPORT  (frame: origin = pivot, axle along X)
    #    Beam from z = -(h_pivot + socket_d) (socket floor) up to z = +top_mrg
    # =====================================================================
    support = (cq.Workplane("XY").box(beam_x, beam_y, beam_h)
                .translate((0, 0, top_mrg - beam_h / 2))
                .faces(">X").workplane(centerOption="ProjectedOrigin")
                .hole(support_hole))                           # through, at the pivot

    # =====================================================================
    # Axle. Frame: origin = pivot, along X, through both supports
    # =====================================================================
    axle_l = 2 * (beam_cx + beam_x / 2 + axle_overhang)
    axle = (cq.Workplane("YZ").circle(axle_d / 2).extrude(axle_l)
            .translate((-axle_l / 2, 0, 0)))

    # =====================================================================
    # Spacer. Tube on the axle between the hub and each beam; keeps the
    # weight from sliding into a beam. Frame: along X, from x = 0 to spacer_l
    # =====================================================================
    opening  = 2 * (beam_cx - beam_x / 2)                      # between the beams' inner faces
    spacer_l = (opening - arm_t - spacer_play) / 2             # one spacer each side
    spacer = (cq.Workplane("YZ")
                .circle(hub_d / 2).circle(arm_hole / 2)        # outer circle + hole -> tube
                .extrude(spacer_l))

    # =====================================================================
    # 7. WALL LOOPS (solid) AND CORE (infill) FOR EVERY PART
    # =====================================================================
    parts = {"base": plate, "support": support, "arm": arm, "weight": weight, "axle": axle,
            "spacer": spacer}
    walls, core = {}, {}
    for name, part in parts.items():
        walls[name] = part.shell(-t_wall)       # walls inside every surface, incl. holes/pockets
        core[name]  = part.cut(walls[name])     # the rest = infill

    # =====================================================================
    # 7b. MASS PROPERTIES (walls at full density, core at density * infill)
    # =====================================================================
    def geom_props(wp):
        """Volume, centre and inertia of all solids in wp, per unit density."""
        p = GProp_GProps()
        BRepGProp.VolumeProperties_s(cq.Compound.makeCompound(wp.solids().vals()).wrapped, p)
        return p

    props = {}
    for name in parts:
        rho = density[filament[name]] * 1e-3              # g/cm^3 -> g/mm^3
        p = GProp_GProps()
        p.Add(geom_props(walls[name]), rho)                # wall loops: solid plastic
        p.Add(geom_props(core[name]),  rho * infill[name]) # core: infill
        props[name] = p
        print(f"{name:8s} {p.Mass():7.2f} g  x{copies[name]}")

    # Filament use per filament (for the Spoolman stock check)
    use = {}
    for name, p in props.items():
        use[filament[name]] = use.get(filament[name], 0.0) + copies[name] * p.Mass()
    print("filament use:", {k: round(v, 1) for k, v in use.items()}, "g")

    # =====================================================================
    # 8. ASSEMBLY AND CHECKS
    # =====================================================================
    support_R  = support.translate(( beam_cx, 0, pivot_z))
    support_L  = support.translate((-beam_cx, 0, pivot_z))
    arm_asm    = arm.translate((0, 0, pivot_z))
    weight_asm = weight.translate((0, 0, pivot_z))
    axle_asm   = axle.translate((0, 0, pivot_z))
    x0         = arm_t / 2 + spacer_play / 2                     # spacer starts just outside the hub
    spacer_R   = spacer.translate(( x0, 0, pivot_z))             # x0 ... x0 + spacer_l
    spacer_L   = spacer.translate((-x0 - spacer_l, 0, pivot_z))  # -(x0 + spacer_l) ... -x0
    total_h = base_t + h_pivot + top_mrg                # plate bottom to support top
    print(f"h_pivot = {h_pivot:.2f} mm   beam_cx = {beam_cx:.2f} mm   "
        f"beam_h = {beam_h:.2f} mm   base_l = {base_l:.1f} mm")
    print(f"total height = {total_h:.1f} mm (H_max = {H_max:.0f}): "
        f"{'OK' if total_h <= H_max else 'TOO TALL'}")

    #show(plate, support_L, support_R, arm_asm, weight_asm, axle_asm, spacer_L, spacer_R)

    # =====================================================================
    # 9. EXPORT TO MUJOCO  (world frame = pivot frame: origin at the axle, axle along X, Z up)
    # =====================================================================
    import os
    os.makedirs("sim_model/meshes", exist_ok=True)

    # Every part placed in the pivot frame (the plate frame shifted down by pivot_z)
    meshes = {
        "base":      plate.translate((0, 0, -pivot_z)),
        "support_R": support.translate(( beam_cx, 0, 0)),
        "support_L": support.translate((-beam_cx, 0, 0)),
        "axle":      axle,
        "spacer_R":  spacer.translate(( x0, 0, 0)),
        "spacer_L":  spacer.translate((-x0 - spacer_l, 0, 0)),
        "arm":       arm,
        "weight":    weight,
    }
    for name, wp in meshes.items():
        cq.exporters.export(wp, f"sim_model/meshes/{name}.stl")

    # Mass properties of the swinging body (arm + weight), from the density model
    swing = GProp_GProps()
    swing.Add(props["arm"])
    swing.Add(props["weight"])
    m_kg = swing.Mass() * 1e-3                                   # g -> kg
    c    = swing.CentreOfMass()                                  # mm
    J    = swing.MatrixOfInertia()                               # about the centre of mass, g*mm^2
    com  = f"{c.X()*1e-3:.6g} {c.Y()*1e-3:.6g} {c.Z()*1e-3:.6g}"           # mm -> m
    dia  = " ".join(f"{J.Value(i, i)*1e-9:.6g}" for i in (1, 2, 3))         # g*mm^2 -> kg*m^2
    print("off-diagonal (should be ~0):", J.Value(1, 2), J.Value(1, 3), J.Value(2, 3))

    filamentC = {
            1 : "0 0 1 1", #red
            2 : "0.5 0.5 0.5 1", #gray
            3 : "0.9 0.6 0.2 1", #blue
            4 : "1 1 1 1", #white
            5 : "0.2 0.2 0.2 1", #black
            6 : "0.4 0.9 0.95 1", #yellow
            7 : "0.9 0.6 0.2 1", #blue
            8 : "0.2 0.2 0.2 1", #black
        }
    
    colors = {
        "base": filamentC.get(params.get("filament_base", "0 0 0.8 1"), params.get("filament_base", "0 0 0.8 1")),
        "support_R": filamentC.get(params.get("filament_stands", "0.8 0.8 0.8 1"), params.get("filament_stands", "0.8 0.8 0.8 1")),
        "support_L": filamentC.get(params.get("filament_stands", "0.8 0.8 0.8 1"), params.get("filament_stands", "0.8 0.8 0.8 1")),
        "axle": filamentC.get(params.get("filament_axle", "0.8 0.8 0.8 1"), params.get("filament_axle", "0.8 0.8 0.8 1")),
        "spacer_R": filamentC.get(params.get("filament_stands", "0.8 0.8 0.8 1"), params.get("filament_stands", "0.8 0.8 0.8 1")),
        "spacer_L": filamentC.get(params.get("filament_stands", "0.8 0.8 0.8 1"), params.get("filament_stands", "0.8 0.8 0.8 1")),
        "arm": filamentC.get(params.get("filament_arm", "0.2 0.6 0.9 1"), params.get("filament_arm", "0.2 0.6 0.9 1")),
        "weight": filamentC.get(params.get("filament_weight", "0.9 0.6 0.2 1"), params.get("filament_weight", "0.9 0.6 0.2 1"))
    }
    
    print(colors["arm"], colors["weight"])
    assets = "\n".join(f'    <mesh name="{n}" file="{n}.stl" scale="0.001 0.001 0.001"/>' for n in meshes)
    static = "\n".join(f'    <geom mesh="{n}" rgba="{colors[n]}"/>' for n in meshes if n not in ("arm", "weight"))

    distance = 1/320*(L-220) + 1
    camera_coordinates = f"{0.5*distance} {-0.5*distance} {-0.05*distance}"
    camera = f"  <camera name=\"cam\" pos=\"{camera_coordinates}\" xyaxes=\"1 1 0  0 0 1\"/>"
    #xyaxes="1 0 0 0 0 -1"
    xml = f"""<mujoco model="pendulum_kit">
    <compiler meshdir="meshes"/>
    <option timestep="0.0001" gravity="0 0 -9.81" integrator="implicitfast"/>
    <asset>
    {assets}
    </asset>
    <default>
        <geom type="mesh" contype="0" conaffinity="0"/>
    </default>
    <worldbody>
        <light pos="0 -1 2" dir="0 0.5 -1"/>
        {camera}

    {static}
        <body name="pendulum">
        <joint name="axle" type="hinge" axis="1 0 0"/>
        <inertial pos="{com}" mass="{m_kg:.6g}" diaginertia="{dia}"/>
        <geom mesh="arm" rgba="{colors['arm']}"/>
        <geom mesh="weight" rgba="{colors['weight']}"/>
        </body>
    </worldbody>
    </mujoco>
    """
    with open("sim_model/pendulum.xml", "w") as f:
        f.write(xml)
    #cq.exporters.export("arm.STL", arm)
    import mujoco
    import mujoco.viewer
    import mujoco.renderer





    model = mujoco.MjModel.from_xml_path("sim_model/pendulum.xml")
    b = model.body("pendulum").id
    print(model.body_mass[b], model.body_ipos[b])   # should equal m_kg and com
    #pendulum.exporters.export(arm, "sim_model/meshes/arm.STL")
    d = mujoco.MjData(model)

    # Create a camera and set its parameters

    #with mujoco.viewer.launch_passive(model, d) as viewer:
    #    viewer.cam.lookat[:] = [0, 0, 0.8]  # point the camera at this position
    #    viewer.cam.distance = 1.0
    #    viewer.cam.azimuth = 90
    #    viewer.cam.elevation = -20
    #
    #    while viewer.is_running():
    #        mujoco.mj_step(model, d)
    #        viewer.sync()


    # Start the pendulum at 90 degrees
    d.qpos[0] = np.pi / 0.99  # 90° in radians
    height = 480
    width = 640
    fps = 30
    # Simulate
    #cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam")
    renderer = mujoco.Renderer(model, height=height, width=width)
    time, angle, angular_velocity = [], [], []
    frames = []
    friction = 0.006*m_kg  # friction torque coefficient
    while d.time < 10.0:
        mujoco.mj_step(model, d)
        time.append(d.time)
        angle.append(d.qpos[0])
        angular_velocity.append(d.qvel[0])
        d.qfrc_applied[0] = -friction * d.qvel[0]  # apply friction torque
        if len(frames) < d.time * fps:
            renderer.update_scene(d,camera="cam")
            frames.append(renderer.render().copy())
    renderer.close()


    video_name = f"..\Flask\static\Pendulum_{order_id}.mp4"
    cap = frames #cv2.VideoCapture(r'C:\Users\edvin\Documents\mas417programmering\video.mp4')
    fourcc = cv2.VideoWriter_fourcc(*'avc1')  # Codec
    video = cv2.VideoWriter(video_name, fourcc, fps, (width,height))
    for frame in frames:
        video.write(frame)
    video.release()

    peak_angle_s = [time[i] for i in range(1, len(angle)-1) if angle[i-1] < angle[i] > angle[i+1]]
    periods = [peak_angle_s[i+1] - peak_angle_s[i] for i in range(len(peak_angle_s)-1)]
    print("Peak angles:", [angle[time.index(t)] for t in peak_angle_s])
    print("Periods:", periods)
    print("Peak angles times:", peak_angle_s)
    plott = 0
    period_show = periods[3] if len(periods) > 3 else periods[len(periods)-1] if len(periods) > 0 else None
    print("Period to show:", period_show)
    if plott:
        plt.plot(time, angle, time ,angular_velocity,peak_angle_s, [angle[time.index(t)] for t in peak_angle_s], 'ro')
        plt.xlabel("Time (s)")
        plt.ylabel("Pendulum Angle (rad) / Angular Velocity (rad/s)")
        plt.title("Pendulum Simulation")
        plt.legend(["Angle", "Angular Velocity", "Peak Angles"])
        plt.show()

    return video_name
