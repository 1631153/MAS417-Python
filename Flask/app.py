#app.py from https://github.com/kubracontractor
#Used as a starting point

from flask import Flask, request, jsonify, render_template
from database import get_connection, create_tables
from functools import wraps
import requests
import math
import os
import json
import shutil
import glob

from pathlib import Path
import sys

# .parent is 'subfolder', .parent.parent is 'project' (one level up)
base = Path(__file__).resolve().parent.parent
MoCoJo_folder = base / "CAD+SIM" 
sys.path.append(str(MoCoJo_folder))

from parts_v3 import simulate_pendulum, estimate_pendulum_summary

base2 = Path(__file__).resolve().parent.parent
PARAMS_folder = base2 / "CAD" 
sys.path.append(str(PARAMS_folder))
from make_params import make_params

app = Flask(__name__)

create_tables()

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

    #Calculate the filament weight consumption for each part
    part_spools = {
        "base": filament_base,
        "support": filament_stands,
        "arm": filament_arm,
        "weight": filament_weight,
        "axle": filament_axle,
        "spacer": filament_axle,
    }

    try:
        estimated_summary = estimate_pendulum_summary(data)
    except Exception as e:
        return jsonify({'error': f'CAD estimation error: {str(e)}'}), 500

    estimated_consume = {}
    for part, spool_id in part_spools.items():
        if spool_id:
            spool_id = int(spool_id)
            estimated_consume[spool_id] = estimated_consume.get(spool_id, 0) + estimated_summary[part].total_mass_g

    #Knowing if the spool has enough filament to consume
    for spool_id, ngrams in estimated_consume.items():
        try:
            res = requests.get(f"http://localhost:7912/api/v1/spool/{spool_id}")
            if res.status_code != 200:
                print(f"Error fetching spool {spool_id}")
    
            spool = res.json()
            remaining_weight = spool.get('remaining_weight', 0)
            if remaining_weight < ngrams:
                return jsonify({'error': f'Not enough filament in spool {spool_id}'}), 400
        except requests.RequestException as e:
            return jsonify({'error': f'Error fetching spool {spool_id}: {str(e)}'}), 500

    #If there is enough filament, we can save the ticket in the database and run the simulation      
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

    delta = None
    summary = {}
    try:
        sim_params = data.copy()
        #make_params(ticket_id, sim_params) #IF YOU WANT TO SAVE THE PARAMS.JSON FILE, UNCOMMENT THIS LINE
        sim_params['order_id'] = ticket_id
        _, summary, delta = simulate_pendulum(sim_params)
    except Exception as e:
        print(f"MuJoCo error: {str(e)}")
    
    spool_consume = {}
    for part, spool_id in part_spools.items():
        if spool_id:
            spool_id = int(spool_id)
            grams = summary[part].total_mass_g
            spool_consume[spool_id] = spool_consume.get(spool_id, 0) + grams
        
    
    for name, item in summary.items():
        print(f"THE SUMMARY HAS BEEN PASSED CORRECTLY: {name}: {item.mass_g} g x {item.copies} = {item.total_mass_g} g")

    #UNCOMMENT THIS TO UPDATE THE FILAMENTS WEIGHT

    for spool_id, grams_used in spool_consume.items():
        try:
            print(f"Updating spool {spool_id} with {grams_used} grams used")
            requests.put(f"http://localhost:7912/api/v1/spool/{spool_id}/use", json={'use_weight': round(grams_used, 2)})
        except requests.RequestException as e:
            print(f"Error updating spool {spool_id}: {str(e)}")
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

#DELETE A TICKET
@app.route('/tickets/<int:ticket_id>', methods=['DELETE'])
def delete_ticket(ticket_id):
    video_path = os.path.join("static", f"Pendulum_{ticket_id}*.mp4")

    for video_path in glob.glob(video_path):
        try:
            os.remove(video_path)
        except Exception as e:
            print(f"Error deleting video file: {str(e)}")

    folder = f"sim_model_{ticket_id}"
    sim_path = os.path.join("SIM_models", folder)
    if os.path.exists(sim_path):
        try:
            shutil.rmtree(sim_path)
        except Exception as e:
            print(f"Error deleting simulation folder: {str(e)}")

    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('DELETE FROM tickets WHERE id = ?', (ticket_id,))
    conn.commit()
    conn.close()

    return jsonify({'message': 'Ticket deleted'}), 200



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