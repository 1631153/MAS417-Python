"""Merge one order from the GUI into params.json.

The GUI's order (the form data sent to POST /tickets, or a row from the tickets table)
is combined with the defaults (config/defaults.json) and the spool data from Spoolman,
and written to  orders/<ticket_id>/params.json.

Use from app.py:      make_params(ticket_id, data)
Use from a terminal:  python make_params.py            (all tickets in the database)
                      python make_params.py 7          (only ticket 7)
"""
import json
import sys
from pathlib import Path

import requests

ROOT = Path(__file__).resolve().parent.parent   # the repository's top folder (this file is in CAD/)SPOOLMAN = "http://localhost:7912"

# GUI field name  ->  part name in params.json
FILAMENT_FIELDS = {
    "base":    "filament_base",
    "support": "filament_stands",
    "arm":     "filament_arm",
    "weight":  "filament_weight",
    "axle":    "filament_axle",
}


def spool_info(sid):
    """Name, material, density, colour and remaining weight of one spool, from Spoolman."""
    res = requests.get(f"{SPOOLMAN}/api/v1/spool/{sid}", timeout=5)
    res.raise_for_status()
    spool = res.json()
    fil = spool["filament"]
    return {
        "name":        fil.get("name"),
        "material":    fil.get("material"),
        "density":     fil["density"],                 # g/cm^3
        "color_hex":   fil.get("color_hex"),
        "remaining_g": spool.get("remaining_weight"),
    }


def make_params(ticket_id, order):
    """Write orders/<ticket_id>/params.json from one GUI order (a dict). Returns its path."""
    defaults  = json.loads((ROOT / "config" / "defaults.json").read_text())
    order_dir = ROOT / "orders" / str(ticket_id)
    order_dir.mkdir(parents=True, exist_ok=True)

    # Spool ID per part; the spacer is not chosen by the customer, so it uses the axle's spool
    filaments = {part: int(order[field]) for part, field in FILAMENT_FIELDS.items()}
    filaments["spacer"] = filaments["axle"]

    # The customer's infill (in %) applies to base, stands and arm;
    # weight, axle and spacer keep the defaults (100 %)
    infill = dict(defaults["print"]["infill"])
    for part in ("base", "support", "arm"):
        infill[part] = float(order["infill"]) / 100.0

    params = {
        "order_id": str(ticket_id),
        "status":   "RECEIVED",
        "order": {
            "customer":  order.get("name"),
            "created":   order.get("creation_date"),
            "T_target":  float(order["swing_period"]),
            "H_max":     float(order["max_size"]),
            "axle_d":    float(order["axle_diameter"]),
            "filaments": filaments,
        },
        "material": {
            "spools": {str(sid): spool_info(sid) for sid in sorted(set(filaments.values()))},
        },
        "print": {**defaults["print"], "n_walls": int(order["walls"]), "infill": infill},
        "design": defaults["design"],
        "solved": {},
        "output": {},
    }

    # Write safely: to a temporary file first, then replace
    path = order_dir / "params.json"
    tmp  = order_dir / "params.json.tmp"
    tmp.write_text(json.dumps(params, indent=2))
    tmp.replace(path)
    return path


if __name__ == "__main__":
    # Create params.json for tickets already in the GUI's database
    sys.path.insert(0, str(ROOT / "Flask"))
    from database import get_connection

    conn = get_connection()
    if len(sys.argv) > 1:
        rows = conn.execute("SELECT * FROM tickets WHERE id = ?", (int(sys.argv[1]),)).fetchall()
    else:
        rows = conn.execute("SELECT * FROM tickets").fetchall()
    conn.close()

    for row in rows:
        ticket = dict(row)
        print("written:", make_params(ticket["id"], ticket))