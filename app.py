from flask import Flask, render_template, request, jsonify
from geometry_msgs.msg import Point32
from utils import gps_to_local_xy, tsp_solver
import rospy
from genetics_algo.gen_test import evolution_loop
import threading

app = Flask(__name__)
# PROBLEMA - NA SIMULAÇÃO, O DRONE NÃO ESTÁ COMEÇANDO EM 0 0!
idx = 0
path = None

live_lat = None
live_lon = None
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/submit_points', methods=['POST'])
def submit_points():
    global path, idx
    data = request.get_json()
    # print("Pontos recebidos:", data)
    
    local_path = []
    if (idx == 0): #pega apenas a posição inicial do drone
        origin_lat = data['droneLatLng']['lat'] 
        origin_lon = data['droneLatLng']['lng']
        idx = 1
        
    # print(origin_lat)
    
    for point in data['payload']:
        x, y = gps_to_local_xy(point['lat'], point['lng'], origin_lat, origin_lon)
        local_path.append({'name': point['species'], 'x':x,'y':y,'z':0.0})
    
    # path = tsp_solver(local_path)
    path = local_path
    print(path)
    return jsonify({'status': 'success'}), 200

@app.route('/get_path', methods=['GET'])
def get_path():
    global path
    if path is None:
        return jsonify({'available': False})
    else:
        return jsonify({
            'available': True,
            'path': path
        })
@app.route('/submit_polygon', methods = ['POST'])
def submit_polygon():
    data = request.get_json()
    print(data)
    origin_lat = data['polygon'][0]['lat']
    origin_lon =  data['polygon'][0]['lng']
    points = []
    for point in data['polygon']:
        x,y = gps_to_local_xy(point['lat'], point['lng'], origin_lat, origin_lon)
        points.append((x,y))
    print(points)
    thread = threading.Thread(target=evolution_loop, args=(points, data['species']))
    thread.start()
    return jsonify({'status': 'success'}), 200

if __name__ == '__main__':
    # rospy.init_node("drone_planter", anonymous=True)
    app.run(debug=True, host='0.0.0.0', port=5000)
