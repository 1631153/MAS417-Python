#app.py from https://github.com/kubracontractor
#Used as a starting point

from flask import Flask, request, jsonify, render_template
from database import get_connection, create_tables
from functools import wraps
from config import API_KEY
import requests
import math
import os
import json

app = Flask(__name__)

create_tables()

#API KEY AUTHENTICATION
def require_api_key(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        key = request.headers.get('X-API-Key')
        if key != API_KEY:
            return jsonify({'error': 'Unauthorized - Invalid or missing API key'}), 401
        return f(*args, **kwargs)
    return decorated


@app.route("/")
def index():
    connect = get_connection()
    cursos = connect.cursor()
    cursos.execute("SELECT * FROM tickets")
    tickets = []
    for row in cursos.fetchall():
        tickets.append(dict(row))
    connect.close()

    filamentColors = {
            1 : "🔴",
            2 : "🔘",
            3 : "🔵",
            4 : "⚪",
            5 : "⚫",
            6 : "🟡",
            7 : "🔵",
            8 : "⚫",
        }

    for ticket in tickets:
        ticket['filament_base_color'] = filamentColors.get(ticket['filament_base'], ticket['filament_base'])
        ticket['filament_stands_color'] = filamentColors.get(ticket['filament_stands'], ticket['filament_stands'])
        ticket['filament_arm_color'] = filamentColors.get(ticket['filament_arm'], ticket['filament_arm'])
        ticket['filament_weight_color'] = filamentColors.get(ticket['filament_weight'], ticket['filament_weight'])
        ticket['filament_axle_color'] = filamentColors.get(ticket['filament_axle'], ticket['filament_axle'])


    return render_template("index.html", tickets=tickets)

#UPDATE FILAMENT WEIGHT (I had the idea, but I'm not really a mathematician, so this is made with Gemini)
#----------------------CHANGE FOR REAL FORMULA (OTHER FILE)----------------------------
def update_filament_weight(max_size_mm, axle_diameter_mm, infill_pct, walls_count, densidad=1.24):
    
    #Convert mm to cm to work directly in cm³ (1 cm³ * density = grams)
    size_cm = (max_size_mm or 100) / 10.0
    axle_r_cm = ((axle_diameter_mm or 8) / 10.0) / 2.0

    #Plastic occupancy factor (walls + internal infill; maximum 1.0 = 100% solid)
    #Assumes a standard extrusion width of 0.4 mm (0.04 cm)
    factor_walls = min(0.6, (walls_count or 2) * 0.08)
    factor_infill = ((infill_pct or 15) / 100.0) * (1.0 - factor_walls)
    factor_solido = min(1.0, factor_walls + factor_infill)

    #Approximate gross volumes in cm³ (Example based on typical proportions)
    vol_base = (size_cm * (size_cm * 0.6) * 0.8)
    vol_stands = 2 * (size_cm * 1.5 * 0.6)
    vol_arm = (size_cm * 1.2 * 0.5)
    vol_weight = math.pi * ((size_cm * 0.25) ** 2) * 1.5
    vol_axle = math.pi * (axle_r_cm ** 2) * (size_cm * 0.5)

    #Convert gross volume to actual grams
    return {
        "base": round(vol_base * factor_solido * densidad, 2),
        "stands": round(vol_stands * factor_solido * densidad, 2),
        "arm": round(vol_arm * factor_solido * densidad, 2),
        "weight": round(vol_weight * factor_solido * densidad, 2),
        "axle": round(vol_axle * factor_solido * densidad, 2),
    }

#CREATE A TICKET
@app.route('/tickets', methods=['POST'])
def create_ticket():
    data = request.get_json()
    
    name = data.get('name')
    swing_period = data.get('swing_period')
    max_size = data.get('max_size')
    axle_diameter = data.get('axle_diameter')
    infill = data.get('infill')
    walls = data.get('walls')
    filament_base = data.get('filament_base')
    filament_stands = data.get('filament_stands')
    filament_arm = data.get('filament_arm')
    filament_weight = data.get('filament_weight')
    filament_axle = data.get('filament_axle')
    creation_date = data.get('creation_date')

    if not name:
        return jsonify({'error': 'Name is required'}), 400

    tests_dir = os.path.join(app.root_path, "json_files")
    os.makedirs(tests_dir, exist_ok=True)
    filename = f"ticket_{name.replace(' ', '_')}.json"
    file_path = os.path.join(tests_dir, filename)

    with open(file_path, 'w') as f:
        json.dump(data, f, indent=4)

    #Calculate the filament weight consumption for each part
    weight = update_filament_weight(max_size, axle_diameter, infill, walls)
    spool_consume = {}
    assignments = [
        (filament_base, weight['base']),
        (filament_stands, weight['stands']),
        (filament_arm, weight['arm']),
        (filament_weight, weight['weight']),
        (filament_axle, weight['axle'])
    ]

    for spool_id, grams in assignments:
        if spool_id:
            spool_id = int(spool_id)
            spool_consume[spool_id] = spool_consume.get(spool_id, 0) + grams
    
    #Knowing if the spool has enough filament to consume
    for spool_id, ngrams in spool_consume.items():
        try:
            res = requests.get(f"http://localhost:7912/api/v1/spool/{spool_id}")
            if res.status_code != 200:
                return jsonify({'error': f'Failed to fetch spool {spool_id}'}), 500

            spool = res.json()
            remaining_weight = spool.get('remaining_weight', 0)
            if remaining_weight < ngrams:
                return jsonify({'error': f'Not enough filament in spool {spool_id}'}), 400
        except requests.RequestException as e:
            return jsonify({'error': f'Error fetching spool {spool_id}: {str(e)}'}), 500
          
    conn = get_connection()
    cursor = conn.cursor()
    
    cursor.execute('''
        INSERT INTO tickets (name, swing_period, max_size, axle_diameter, infill, walls, 
                             filament_base, filament_stands, filament_arm, filament_weight, filament_axle, creation_date)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    ''', (name, swing_period, max_size, axle_diameter, infill, walls, 
          filament_base, filament_stands, filament_arm, filament_weight, filament_axle, creation_date))
          
    conn.commit()
    ticket_id = cursor.lastrowid
    conn.close()

    for spool_id, grams_used in spool_consume.items():
        try:
            requests.put(f"http://localhost:7912/api/v1/spool/{spool_id}/use", json={'use_weight': round(grams_used, 2)})
        except requests.RequestException as e:
            return jsonify({'error': f'Error updating spool {spool_id}: {str(e)}'}), 500

    return jsonify({'message': 'Ticket created', 'id': ticket_id, 'discounted_grams': spool_consume}), 201

@app.route('/ticket_form')
def ticket_form():
    response = requests.get('http://localhost:7912/api/v1/spool')
    if response.status_code == 200:
        filaments = response.json() 
    else:
        filaments = []
        print("Error fetching filaments")

    filamentColors = {
        "FF0000" : "🔴",
        "808080" : "🔘",
        "0080FF" : "🔵",
        "FFFFFF" : "⚪",
        "000000" : "⚫",
        "FFFF80" : "🟡",
    }

    for filament in filaments:
        filament['color_name'] = filamentColors.get(filament['filament']['color_hex'], filament['filament']['color_hex'])
    
    return render_template("ticket_form.html", filaments=filaments)

#GET ALL TICKETS 
@app.route('/tickets', methods=['GET'])
def get_tickets():
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM tickets')
    tickets = [dict(row) for row in cursor.fetchall()]
    conn.close()
    return jsonify(tickets), 200



#---------------------------------------------------------------
#GET ONE TICKET
@app.route('/tickets/<int:ticket_id>', methods=['GET'])
def get_ticket(ticket_id):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM tickets WHERE id = ?', (ticket_id,))
    ticket = cursor.fetchone()
    conn.close()

    if ticket is None:
        return jsonify({'error': 'Ticket not found'}), 404

    return jsonify(dict(ticket)), 200


#UPDATE A TICKET
@app.route('/tickets/<int:ticket_id>', methods=['PUT'])
def update_ticket(ticket_id):
    data = request.get_json()
    priority = data.get('priority')
    status = data.get('status')

    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('''
        UPDATE tickets SET priority = ?, status = ?
        WHERE id = ?
    ''', (priority, status, ticket_id))
    conn.commit()
    conn.close()

    return jsonify({'message': 'Ticket updated'}), 200


#DELETE A TICKET
@app.route('/tickets/<int:ticket_id>', methods=['DELETE'])
def delete_ticket(ticket_id):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('DELETE FROM tickets WHERE id = ?', (ticket_id,))
    conn.commit()
    conn.close()

    return jsonify({'message': 'Ticket deleted'}), 200

#ERROR HANDLERS
@app.errorhandler(404)
def not_found(error):
    return jsonify({'error': 'Resource not found'}), 404

@app.errorhandler(400)
def bad_request(error):
    return jsonify({'error': 'Bad request'}), 400

@app.errorhandler(500)
def internal_error(error):
    return jsonify({'error': 'Internal server error'}), 500
if __name__ == '__main__':
    app.run(debug=True)