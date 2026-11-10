#app.py from https://github.com/kubracontractor
#Used as a starting point

from flask import Flask, request, jsonify, render_template
from database import get_connection, create_tables
from functools import wraps
from config import API_KEY

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

    return jsonify({'message': 'Ticket created', 'id': ticket_id}), 201

@app.route('/ticket_form')
def ticket_form():
    return render_template("ticket_form.html")



#GET ALL TICKETS 
@app.route('/tickets', methods=['GET'])
def get_tickets():
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM tickets')
    tickets = [dict(row) for row in cursor.fetchall()]
    conn.close()
    return jsonify(tickets), 200


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