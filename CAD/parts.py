import json
import sys
from pathlib import Path

import math

import cadquery as cq
from cadquery.vis import show

from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Ax1, gp_Pnt, gp_Dir

# =====================================================================
# 1. PARAMETERS  (from params.json)
# =====================================================================
params_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent / "test.params.json"
with open(params_path) as f:
    params = json.load(f)
order_dir = params_path.parent

# Customer order
axle_d   = params["order"]["axle_d"]
H_max    = params["order"]["H_max"]

# Print settings
n_walls  = params["print"]["n_walls"]
line_w   = params["print"]["line_w"]
infill   = params["print"]["infill"]

# Material: filament (spool ID) per part, density per spool (from Spoolman, g/cm^3)
filament = {part: str(sid) for part, sid in params["order"]["filaments"].items()}
filament.setdefault("spacer", filament["axle"])        # if the GUI doesn't set the spacer
density  = {sid: s["density"] for sid, s in params["material"]["spools"].items()}
copies   = {"base": 1, "support": 2, "arm": 1, "weight": 1, "axle": 1, "spacer": 2}

# Fits and clearances [mm]
d = params["design"]
fit           = d["fit"]
support_clr   = d["support_clr"]
arm_clr       = d["arm_clr"]
press         = d["press"]
gap           = d["gap"]
c_base        = d["c_base"]
axle_overhang = d["axle_overhang"]
spacer_play   = d["spacer_play"]

# Arm
arm_t    = d["arm"]["t"]
arm_w    = d["arm"]["w"]
hub_wall = d["arm"]["hub_wall"]

# Weight (the size comes per candidate from "solved")
slot_d   = d["weight"]["slot_d"]

# Supports (beams)
beam_x   = d["beam"]["x"]
beam_y   = d["beam"]["y"]
top_mrg  = d["beam"]["top_mrg"]

# Base plate
base_t      = d["base"]["t"]
base_w      = d["base"]["w"]
base_margin = d["base"]["margin"]
socket_d    = d["base"]["socket_d"]

# Weight sizes with their solved arm lengths (skip any the solver couldn't make work)
candidates = [c for c in params["solved"]["candidates"] if c.get("ok", True)]

# =====================================================================
# 2. DERIVED DIMENSIONS (from the parameters only)
# =====================================================================
t_wall       = n_walls * line_w             # 2 * 0.45 = 0.9 mm
arm_hole     = axle_d + arm_clr             # 5 + 1 = 6.0 mm
hub_d        = arm_hole + 2 * hub_wall      # 6 + 9 = 15.0 mm
support_hole = axle_d + support_clr         # 5 + 0.3 = 5.3 mm
sock_dx      = beam_x + fit                 # 12.3 mm (socket larger than the beam)
sock_dy      = beam_y + fit                 # 25.3 mm

# =====================================================================
# BUILD ONE KIT: sections 3 to 7b for one weight size and its arm length L
# =====================================================================
def build_kit(weight_t, weight_w, weight_h, L):
    z_top = -L + slot_d                            # weight top face in the pivot frame

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

    total_h = base_t + h_pivot + top_mrg               # plate bottom to support top
    total_g = sum(copies[name] * props[name].Mass() for name in props)   # whole kit [g]
    return {"weight_size": (weight_t, weight_w, weight_h), "L": L,
            "total_g": total_g, "total_h": total_h, "valid": total_h <= H_max,
            "plate": plate, "support": support, "arm": arm, "weight": weight,
            "axle": axle, "spacer": spacer, "props": props,
            "h_pivot": h_pivot, "beam_cx": beam_cx, "pivot_z": pivot_z,
            "beam_h": beam_h, "base_l": base_l, "spacer_l": spacer_l}


# =====================================================================
# 7c. BUILD EVERY CANDIDATE, CHOOSE THE LIGHTEST KIT WITHIN H_max
# =====================================================================
kits = []
for c in candidates:
    w = c["weight"]
    print(f"\n--- weight {w['t']} x {w['w']} x {w['h']} mm, L = {c['L']:.2f} mm ---")
    kits.append(build_kit(w["t"], w["w"], w["h"], c["L"]))

print("\nCandidates:")
for k in kits:
    print(f"  weight {k['weight_size']}  L = {k['L']:.1f} mm  total = {k['total_g']:.1f} g  "
          f"height = {k['total_h']:.1f} mm  {'OK' if k['valid'] else 'TOO TALL'}")

valid = [k for k in kits if k["valid"]]
if not valid:
    raise ValueError(f"no weight size gives a kit within H_max = {H_max} mm")
best = min(valid, key=lambda k: k["total_g"])
print(f"Chosen: weight {best['weight_size']} mm, L = {best['L']:.1f} mm, {best['total_g']:.1f} g\n")

# Sections 8 and 9 use the chosen kit
plate, support, arm, weight = best["plate"], best["support"], best["arm"], best["weight"]
axle, spacer, props = best["axle"], best["spacer"], best["props"]
h_pivot, beam_cx, pivot_z = best["h_pivot"], best["beam_cx"], best["pivot_z"]
beam_h, base_l, spacer_l = best["beam_h"], best["base_l"], best["spacer_l"]

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

show(plate, support_L, support_R, arm_asm, weight_asm, axle_asm, spacer_L, spacer_R)

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

assets = "\n".join(f'    <mesh name="{n}" file="{n}.stl" scale="0.001 0.001 0.001"/>' for n in meshes)
static = "\n".join(f'    <geom mesh="{n}"/>' for n in meshes if n not in ("arm", "weight"))

xml = f"""<mujoco model="pendulum_kit">
  <compiler meshdir="meshes"/>
  <option timestep="0.0001" gravity="0 0 -9.81"/>
  <asset>
{assets}
  </asset>
  <default>
    <geom type="mesh" contype="0" conaffinity="0"/>
  </default>
  <worldbody>
{static}
    <body name="pendulum">
      <joint name="axle" type="hinge" axis="1 0 0"/>
      <inertial pos="{com}" mass="{m_kg:.6g}" diaginertia="{dia}"/>
      <geom mesh="arm"/>
      <geom mesh="weight"/>
    </body>
  </worldbody>
</mujoco>
"""
with open("sim_model/pendulum.xml", "w") as f:
    f.write(xml)

import mujoco
model = mujoco.MjModel.from_xml_path("sim_model/pendulum.xml")
b = model.body("pendulum").id
print(model.body_mass[b], model.body_ipos[b])   # should equal m_kg and com